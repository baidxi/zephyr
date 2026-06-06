/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner V3s DE2 Mixer display driver.
 *
 * Implements Zephyr display_driver_api using the DE2 Mixer's UI layer
 * in single-framebuffer mode. References TCON for panel timing and
 * DE2 CCU for clocks/resets.
 */

#define DT_DRV_COMPAT allwinner_sun8i_v3s_de2_mixer

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "tcon_sun8i_v3s.h"
#include "display_sun8i_de2_mixer_regs.h"

LOG_MODULE_REGISTER(de2_mixer, CONFIG_DISPLAY_LOG_LEVEL);

struct de2_fmt_entry {
	enum display_pixel_format zephyr_fmt;
	uint32_t de2_fmt;
};

static const struct de2_fmt_entry de2_fmt_table[] = {
	{ PIXEL_FORMAT_ARGB_8888, DE2_FBFMT_ARGB8888 },
	{ PIXEL_FORMAT_XRGB_8888, DE2_FBFMT_XRGB8888 },
	{ PIXEL_FORMAT_RGB_888,   DE2_FBFMT_RGB888   },
	{ PIXEL_FORMAT_RGB_565,   DE2_FBFMT_RGB565   },
};

static int de2_find_format(enum display_pixel_format zfmt, uint32_t *de2fmt)
{
	for (int i = 0; i < ARRAY_SIZE(de2_fmt_table); i++) {
		if (de2_fmt_table[i].zephyr_fmt == zfmt) {
			*de2fmt = de2_fmt_table[i].de2_fmt;
			return 0;
		}
	}
	return -ENOTSUP;
}

struct de2_mixer_data {
	uintptr_t base;
	uint8_t *framebuffer;	/* App write buffer (stable pointer) */
	uint8_t *fb_display[2];	/* Ping-pong display buffers for DE2 DMA */
	uint8_t fb_active;	/* Index of buffer DE2 is reading (0 or 1) */
	uint32_t fb_size;
	uint16_t width;
	uint16_t height;
	enum display_pixel_format pixel_format;
	const struct device *tcon;
	struct k_mutex lock;
	bool enabled;
};

struct de2_mixer_config {
	uintptr_t base;
	const struct device *clock_dev;
	clock_control_subsys_t bus_clk;
	clock_control_subsys_t mod_clk;
	struct reset_dt_spec reset;
	uint16_t width;
	uint16_t height;
	enum display_pixel_format init_fmt;
};

/* 0x6000 bytes, write 0 in 4-byte increments */
static void de2_reset_clear(uintptr_t base)
{
	/* Disable unused sub-engines */
	sys_write32(0, base + SUN8I_MIXER_FCE_EN);
	sys_write32(0, base + SUN8I_MIXER_BWS_EN);
	sys_write32(0, base + SUN8I_MIXER_LTI_EN);
	sys_write32(0, base + SUN8I_MIXER_PEAK_EN);
	sys_write32(0, base + SUN8I_MIXER_ASE_EN);
	sys_write32(0, base + SUN8I_MIXER_FCC_EN);
	sys_write32(0, base + SUN8I_MIXER_DCSC_EN);

	/* Clear register space */
	for (uint32_t off = 0; off < DE2_MIXER_UNIT_SIZE; off += 4) {
		sys_write32(0, base + off);
	}
}

/*
 * Blender init — bld_off is a *relative offset* from mixer base.
 * All BLEND_* macros take an offset and return an offset, so
 * the final write address is: base + BLEND_*(bld_off, ...).
 */
static void de2_blender_init(uintptr_t base)
{
	uint32_t bld_off = DE2_BLD_BASE;

	sys_write32(0xFF000000, base + BLEND_ATTR_FCOLOR(bld_off, 0));
	sys_write32(0,          base + BLEND_PREMULTIPLY(bld_off));
	sys_write32(0xFF000000, base + BLEND_BKCOLOR(bld_off));
	sys_write32(BLEND_MODE_DEF, base + BLEND_MODE(bld_off, 0));
	sys_write32(0, base + BLEND_CK_CTL(bld_off));
	sys_write32(0, base + BLEND_CK_CFG(bld_off));
	sys_write32(0, base + BLEND_OUTCTL(bld_off));
}

static void de2_set_output_size(uintptr_t base, uint16_t w, uint16_t h)
{
	uint32_t bld_off = DE2_BLD_BASE;
	uint32_t size = SUN8I_MIXER_SIZE(w, h);

	sys_write32(size, base + SUN8I_MIXER_GLOBAL_SIZE);
	sys_write32(size, base + BLEND_OUTSIZE(bld_off));

	/*
	 * Bug #12c fix: set blender pipe 0 input size and coordinates.
	 * Without BLEND_ATTR_INSIZE, the blender has a zero-sized input
	 * window and produces no output for the pipe.
	 * Linux sets these in sun8i_ui_layer_update_coord().
	 */
	sys_write32(size, base + BLEND_ATTR_INSIZE(bld_off, 0));
	sys_write32(0, base + BLEND_ATTR_COORD(bld_off, 0));
}

/*
 * UI layer setup — ch_off is a *relative offset* from mixer base.
 * All CHAN_UI_LAYER_* macros take an offset and return an offset.
 */
static void de2_ui_layer_setup(uintptr_t base,
				uint32_t fb_paddr, uint32_t de2_fmt,
				uint16_t w, uint16_t h, uint16_t pitch_bytes)
{
	uint32_t ch_off = DE2_CH_BASE + MIXER_UI_CHANNEL * DE2_CH_SIZE;

	/*
	 * Layer attribute: enable + format + alpha.
	 * Bug #12d fix: Linux always sets ALPHA = 0xFF in bits [31:24]
	 * via sun8i_ui_layer_update_alpha(), even for pixel alpha mode.
	 * For RGB565 (no per-pixel alpha), the hardware uses this field
	 * as the effective alpha. Without it, ALPHA=0 makes the layer
	 * fully transparent (white screen on LCD).
	 */
	uint32_t attr = LAYER_ATTR_EN
		      | (de2_fmt << LAYER_ATTR_FBFMT_SHIFT)
		      | LAYER_ATTR_ALPHA(0xFF);
	sys_write32(attr, base + CHAN_UI_LAYER_ATTR(ch_off, MIXER_UI_OVERLAY));

	/* Layer size (source = panel size, no scaling) */
	uint32_t size = ((h - 1) << 16) | (w - 1);
	sys_write32(size, base + CHAN_UI_LAYER_SIZE(ch_off, MIXER_UI_OVERLAY));

	/* Layer coordinate (0, 0) */
	sys_write32(0, base + CHAN_UI_LAYER_COORD(ch_off, MIXER_UI_OVERLAY));

	/* Pitch in bytes */
	sys_write32(pitch_bytes, base + CHAN_UI_LAYER_PITCH(ch_off, MIXER_UI_OVERLAY));

	/* Framebuffer physical address */
	sys_write32(fb_paddr, base + CHAN_UI_LAYER_TOP_LADDR(ch_off, MIXER_UI_OVERLAY));

	/* Overlay output size: SUN8I_MIXER_SIZE(w, h) = ((h-1)<<16)|(w-1) */
	sys_write32(((h - 1) << 16) | (w - 1), base + CHAN_UI_OVL_SIZE(ch_off));
}

static void de2_mixer_enable(uintptr_t base)
{
	uint32_t bld_off = DE2_BLD_BASE;

	/*
	 * Bug #12a fix: BLEND_ROUTE selects which channel feeds each pipe.
	 * Each 4-bit field selects the channel for the corresponding pipe.
	 * MIXER_UI_CHANNEL (2) = UI channel for V3s (vi_num=2, so UI=ch2).
	 * Value 2 in bits [3:0] routes channel 2 → pipe 0.
	 * Previously was 0 which routed unconfigured VI channel 0.
	 */
	sys_write32(MIXER_UI_CHANNEL, base + BLEND_ROUTE(bld_off));

	/*
	 * Pipe enable: pipe 0 + FC_EN(0).
	 * Bug #12b fix: Linux always sets FC_EN(0) = BIT(0) alongside
	 * PIPE_EN(0) = BIT(8). FC_EN enables the force-color fallback
	 * for the pipe.
	 */
	sys_write32(BLEND_PIPE_CTL_EN(0) | BLEND_PIPE_CTL_FC_EN(0),
	     base + BLEND_PIPE_CTL(bld_off));

	/*
	 * BLEND_OUTCTL: Bug #18 fix — offset corrected from 0xCC back to
	 * 0xFC (matching Linux SUN8I_MIXER_BLEND_OUTCTL).  For non-interlaced
	 * RGB LCD panels, Linux sun8i_mixer_mode_set() writes 0 here.
	 * Writing INTERLACED (BIT(1)) would be wrong for a progressive panel.
	 * The init code already cleared this register to 0 in
	 * de2_blender_init(); this write ensures the register takes effect
	 * through the double-buffer path.
	 */
	sys_write32(0, base + BLEND_OUTCTL(bld_off));

	/* Global double-buffer enable */
	sys_write32(SUN8I_MIXER_GLOBAL_DBUFF_ENABLE,
		    base + SUN8I_MIXER_GLOBAL_DBUFF);

	/* Global real-time enable */
	sys_write32(SUN8I_MIXER_GLOBAL_CTL_RT_EN,
		    base + SUN8I_MIXER_GLOBAL_CTL);
}

static void de2_mixer_disable(uintptr_t base)
{
	uint32_t bld_off = DE2_BLD_BASE;

	sys_write32(0, base + SUN8I_MIXER_GLOBAL_CTL);
	sys_write32(0, base + BLEND_PIPE_CTL(bld_off));
}

static void de2_get_capabilities(const struct device *dev,
				  struct display_capabilities *caps)
{
	struct de2_mixer_data *data = dev->data;

	memset(caps, 0, sizeof(*caps));
	caps->x_resolution = data->width;
	caps->y_resolution = data->height;
	caps->supported_pixel_formats =
		PIXEL_FORMAT_ARGB_8888 |
		PIXEL_FORMAT_XRGB_8888 |
		PIXEL_FORMAT_RGB_888   |
		PIXEL_FORMAT_RGB_565;
	caps->current_pixel_format = data->pixel_format;
	caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static int de2_set_pixel_format(const struct device *dev,
				 enum display_pixel_format fmt)
{
	struct de2_mixer_data *data = dev->data;
	uint32_t de2fmt;

	if (de2_find_format(fmt, &de2fmt) != 0) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->pixel_format = fmt;
	k_mutex_unlock(&data->lock);
	return 0;
}

static int de2_blanking_on(const struct device *dev)
{
	struct de2_mixer_data *data = dev->data;

	de2_mixer_disable(data->base);
	tcon_enable_output(data->tcon, false);
	data->enabled = false;
	return 0;
}

static int de2_blanking_off(const struct device *dev)
{
	struct de2_mixer_data *data = dev->data;

	de2_set_output_size(data->base, data->width, data->height);
	de2_mixer_enable(data->base);
	tcon_enable_output(data->tcon, true);
	data->enabled = true;
	return 0;
}

static int de2_write(const struct device *dev,
		      const uint16_t x, const uint16_t y,
		      const struct display_buffer_descriptor *desc,
		      const void *buf)
{
	struct de2_mixer_data *data = dev->data;
	uint32_t de2fmt;
	int ret;

	/* Full-screen only in simplified mode */
	if (x != 0 || y != 0) {
		return -ENOTSUP;
	}

	ret = de2_find_format(data->pixel_format, &de2fmt);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Copy to internal write buffer if buf is different */
	if (buf != data->framebuffer && buf != NULL) {
		memcpy(data->framebuffer, buf, data->fb_size);
	}

	/*
	 * Bug #22 fix: Ping-pong double-buffering to prevent tearing.
	 *
	 * The app writes to data->framebuffer (CPU cache).  We copy the
	 * data to the INACTIVE display buffer, flush it to DDR, and point
	 * the DE2 layer at it.  The DE2 is still reading from the ACTIVE
	 * display buffer, so the copy is race-free.
	 *
	 * After the DE2 double-buffer commit, the DE2 switches to the new
	 * (previously inactive) buffer at the next vsync boundary.  The
	 * old active buffer becomes the new inactive buffer for next frame.
	 */
	uint8_t next = 1 - data->fb_active;

	memcpy(data->fb_display[next], data->framebuffer, data->fb_size);

	/*
	 * ARM Cortex-A7 has data cache.  Flush the display buffer to DDR
	 * so the DE2 DMA engine sees the latest pixel data.
	 */
	sys_cache_data_flush_range(data->fb_display[next], data->fb_size);

	/* Physical address (identity-mapped DDR: va == pa) */
	uint32_t fb_paddr = (uint32_t)(uintptr_t)data->fb_display[next];
	uint32_t bpp = DISPLAY_BITS_PER_PIXEL(data->pixel_format) / 8;
	uint16_t pitch = desc ? (desc->pitch * bpp) : (data->width * bpp);

	/* Setup UI layer to read from the new display buffer */
	de2_ui_layer_setup(data->base, fb_paddr, de2fmt,
			    data->width, data->height, pitch);

	/*
	 * Bug #19 fix: commit double-buffered registers every frame.
	 *
	 * Linux sun8i_mixer_commit() writes BLEND_ROUTE, BLEND_PIPE_CTL,
	 * and GLOBAL_DBUFF on EVERY frame (not just the first).  The DE2
	 * hardware uses double-buffered registers: writes go to shadow
	 * registers and only take effect when GLOBAL_DBUFF is written.
	 *
	 * Additionally, the Linux driver notes: "We always update the layer
	 * enable bit, because it can clear spontaneously for unknown reasons."
	 * Without committing GLOBAL_DBUFF each frame, a spontaneously-cleared
	 * layer enable cannot be recovered, causing display flicker.
	 */
	{
		uint32_t bld_off = DE2_BLD_BASE;

		sys_write32(MIXER_UI_CHANNEL,
			    data->base + BLEND_ROUTE(bld_off));
		sys_write32(BLEND_PIPE_CTL_EN(0) | BLEND_PIPE_CTL_FC_EN(0),
			    data->base + BLEND_PIPE_CTL(bld_off));
		sys_write32(SUN8I_MIXER_GLOBAL_DBUFF_ENABLE,
			    data->base + SUN8I_MIXER_GLOBAL_DBUFF);
	}

	/* Swap: the inactive buffer is now the active display buffer */
	data->fb_active = next;

	/* First frame: initialize TCON + enable output.
	 *
	 * Bug #21b fix: match Linux initialization order.
	 * Linux enables the TCON first (via CRTC atomic enable),
	 * then commits DE2 registers (via mixer atomic flush).
	 * Previously we enabled DE2 before TCON, which caused the
	 * DE2 to start outputting data before the TCON was consuming
	 * it, leading to FIFO desync and "random Y start" artifacts.
	 *
	 * Correct order: TCON configure → TCON enable → DE2 enable.
	 */
	if (!data->enabled) {
		struct tcon_timings tm = *tcon_get_timings(data->tcon);
		tm.width  = data->width;
		tm.height = data->height;

		/* 1. Configure TCON timings (does NOT enable output yet) */
		tcon_apply_timings(data->tcon, &tm);

		/* 2. Set DE2 output size (writes GLOBAL_SIZE, BLEND_OUTSIZE) */
		de2_set_output_size(data->base, data->width, data->height);

		/* 3. Enable TCON output FIRST — starts consuming pixel data */
		tcon_enable_output(data->tcon, true);

		/* 4. Enable DE2 mixer — starts producing pixel data.
		 *    TCON is already running and will read from frame start. */
		de2_mixer_enable(data->base);

		data->enabled = true;

		/* Diagnostic: dump DE2 CCU + main CCU registers to diagnose
		 * why mixer registers read as zero */
		uintptr_t de2_ccu_base = 0x01000000;
		uintptr_t main_ccu_base = 0x01c20000;
		LOG_DBG("DE2 CCU  MOD_CLK=0x%08x BUS_CLK=0x%08x RST=0x%08x DIV=0x%08x",
			sys_read32(de2_ccu_base + 0x00),
			sys_read32(de2_ccu_base + 0x04),
			sys_read32(de2_ccu_base + 0x08),
			sys_read32(de2_ccu_base + 0x0c));
		LOG_DBG("Main CCU BUS_DE gate (0x64)=0x%08x  DE_CLK (0x104)=0x%08x",
			sys_read32(main_ccu_base + 0x64),
			sys_read32(main_ccu_base + 0x104));
		LOG_DBG("Main CCU RST_reg (0x2c4)=0x%08x  [bit12=RST_BUS_DE, 1=deasserted]",
			sys_read32(main_ccu_base + 0x2c4));

		/* Force deassert RST_BUS_DE (set bit12 at main CCU 0x2c4) */
		uint32_t rst_reg = sys_read32(main_ccu_base + 0x2c4);
		rst_reg |= BIT(12);
		sys_write32(rst_reg, main_ccu_base + 0x2c4);
		LOG_DBG("After force deassert RST_BUS_DE: 0x2c4=0x%08x",
			sys_read32(main_ccu_base + 0x2c4));
		LOG_DBG("DE2 CCU after RST fix: MOD=0x%08x BUS=0x%08x RST=0x%08x",
			sys_read32(de2_ccu_base + 0x00),
			sys_read32(de2_ccu_base + 0x04),
			sys_read32(de2_ccu_base + 0x08));

		/* Write+readback test on mixer GLOBAL_DBUFF (safe W1C) */
		sys_write32(0xdeadbeef, data->base + SUN8I_MIXER_GLOBAL_DBUFF);
		LOG_DBG("Mixer DBUFF write+readback: wrote=0xdeadbeef read=0x%08x",
			sys_read32(data->base + SUN8I_MIXER_GLOBAL_DBUFF));

		/* Diagnostic: dump key DE2 registers */
		uint32_t ch_off = DE2_CH_BASE + MIXER_UI_CHANNEL * DE2_CH_SIZE;
		uint32_t bld_off = DE2_BLD_BASE;
		LOG_DBG("DE2 GLOBAL_SIZE =0x%08x",
			sys_read32(data->base + SUN8I_MIXER_GLOBAL_SIZE));
		LOG_DBG("DE2 GLOBAL_CTL  =0x%08x",
			sys_read32(data->base + SUN8I_MIXER_GLOBAL_CTL));
		LOG_DBG("DE2 GLOBAL_DBUFF=0x%08x",
			sys_read32(data->base + SUN8I_MIXER_GLOBAL_DBUFF));
		LOG_DBG("DE2 LAYER_ATTR  =0x%08x",
			sys_read32(data->base + CHAN_UI_LAYER_ATTR(ch_off, 0)));
		LOG_DBG("DE2 LAYER_SIZE  =0x%08x",
			sys_read32(data->base + CHAN_UI_LAYER_SIZE(ch_off, 0)));
		LOG_DBG("DE2 LAYER_PITCH =0x%08x",
			sys_read32(data->base + CHAN_UI_LAYER_PITCH(ch_off, 0)));
		LOG_DBG("DE2 LAYER_ADDR  =0x%08x",
			sys_read32(data->base + CHAN_UI_LAYER_TOP_LADDR(ch_off, 0)));
		LOG_DBG("DE2 OVL_SIZE    =0x%08x",
			sys_read32(data->base + CHAN_UI_OVL_SIZE(ch_off)));
		LOG_DBG("DE2 BLEND_ROUTE =0x%08x",
			sys_read32(data->base + BLEND_ROUTE(bld_off)));
		LOG_DBG("DE2 BLEND_PIPE  =0x%08x",
			sys_read32(data->base + BLEND_PIPE_CTL(bld_off)));
		LOG_DBG("DE2 BLEND_OUTSZ =0x%08x",
			sys_read32(data->base + BLEND_OUTSIZE(bld_off)));
		LOG_DBG("DE2 BLEND_INSIZE=0x%08x  [pipe0 input size]",
			sys_read32(data->base + BLEND_ATTR_INSIZE(bld_off, 0)));
		LOG_DBG("DE2 BLEND_COORD =0x%08x  [pipe0 coord]",
			sys_read32(data->base + BLEND_ATTR_COORD(bld_off, 0)));
		LOG_DBG("DE2 BLEND_OUTCTL=0x%08x",
			sys_read32(data->base + BLEND_OUTCTL(bld_off)));

		/* Diagnostic: verify LCD pin mux (PE config registers) */
		/* PE port offset in PIO: 0x24 * 4 = 0x90 */
		uintptr_t pio_base = 0x01C20800;
		uintptr_t pe_base = pio_base + 0x24 * 4; /* PE = port 4 */
		LOG_DBG("PIO PE_CFG0=0x%08x [PE0-7]", sys_read32(pe_base + 0x00));
		LOG_DBG("PIO PE_CFG1=0x%08x [PE8-15]", sys_read32(pe_base + 0x04));
		LOG_DBG("PIO PE_CFG2=0x%08x [PE16-23]", sys_read32(pe_base + 0x08));
		LOG_DBG("PIO PE_CFG3=0x%08x [PE24]", sys_read32(pe_base + 0x0C));

		/* Diagnostic: verify framebuffer data at multiple offsets
		 * Color bars: WHITE(0) YELLOW(60) CYAN(120) GREEN(180)
		 * Each row=800 px, each bar=60 rows, 2 bytes/px
		 * Bar offset in uint32 = (row * 800) / 2 = row * 400
		 */
		if (data->framebuffer) {
			uint32_t *fb32 = (uint32_t *)data->framebuffer;
			LOG_DBG("FB @row0 (WHITE):  %08x %08x", fb32[0], fb32[1]);
			LOG_DBG("FB @row60(YELLOW): %08x %08x",
				fb32[60 * 400], fb32[60 * 400 + 1]);
			LOG_DBG("FB @row120(CYAN):  %08x %08x",
				fb32[120 * 400], fb32[120 * 400 + 1]);
			LOG_DBG("FB @row240(MAGENTA): %08x %08x",
				fb32[240 * 400], fb32[240 * 400 + 1]);
			LOG_DBG("FB @row420(BLACK): %08x %08x",
				fb32[420 * 400], fb32[420 * 400 + 1]);
		}

		/* Diagnostic: GLOBAL_STATUS register */
		LOG_DBG("DE2 GLOBAL_STATUS=0x%08x",
			sys_read32(data->base + SUN8I_MIXER_GLOBAL_STATUS));

		/* Diagnostic: TCON0 registers (base 0x01c0c000) */
		uintptr_t tcon0 = 0x01c0c000;
		uint32_t dclk_reg = sys_read32(tcon0 + 0x44);
		LOG_DBG("TCON GCTL  =0x%08x [exp 0x80000000]",
			sys_read32(tcon0 + 0x00));
		LOG_DBG("TCON0 CTL  =0x%08x [exp 0x800000XX]",
			sys_read32(tcon0 + 0x40));
		LOG_DBG("TCON0 DCLK =0x%08x [LCLK_EN=%u DCLKDIV=%u]",
			dclk_reg, (dclk_reg >> 28) & 0xF, dclk_reg & 0x7F);
		LOG_DBG("TCON0 B0   =0x%08x [res]",
			sys_read32(tcon0 + 0x48));
		LOG_DBG("TCON0 B1   =0x%08x [h timing]",
			sys_read32(tcon0 + 0x4C));
		LOG_DBG("TCON0 B2   =0x%08x [v timing]",
			sys_read32(tcon0 + 0x50));
		LOG_DBG("TCON0 B3   =0x%08x [sync]",
			sys_read32(tcon0 + 0x54));
		LOG_DBG("TCON0 IO_POL=0x%08x", sys_read32(tcon0 + 0x88));
		LOG_DBG("TCON0 IO_TRI=0x%08x [exp 0x00000000]",
			sys_read32(tcon0 + 0x8C));

		/* Diagnostic: DE2 BLEND_OUTCTL at corrected offset 0x10FC
		 * (Bug #18: was 0x10CC). For non-interlaced LCD, expect 0x00.
		 */
		LOG_DBG("DE2 BLEND_OUTCTL=0x%08x [exp 0x00, offset 0x10FC]",
			sys_read32(data->base + BLEND_OUTCTL(DE2_BLD_BASE)));
	}

	k_mutex_unlock(&data->lock);
	return 0;
}

static void *de2_get_framebuffer(const struct device *dev)
{
	struct de2_mixer_data *data = dev->data;

	return data->framebuffer;
}

static int de2_set_brightness(const struct device *dev, uint8_t brightness)
{
	struct de2_mixer_data *data = dev->data;

	return tcon_set_brightness(data->tcon, brightness);
}

static const struct display_driver_api de2_mixer_api = {
	.blanking_on      = de2_blanking_on,
	.blanking_off     = de2_blanking_off,
	.write            = de2_write,
	.get_framebuffer  = de2_get_framebuffer,
	.get_capabilities = de2_get_capabilities,
	.set_pixel_format = de2_set_pixel_format,
	.set_brightness   = de2_set_brightness,
};

static int de2_mixer_init(const struct device *dev)
{
	const struct de2_mixer_config *cfg = dev->config;
	struct de2_mixer_data *data = dev->data;
	int ret;

	/* ---- 1. Initialize base address from config ---- */
	data->base = cfg->base;

	/* ---- 2. TCON ---- */
	data->tcon = DEVICE_DT_GET(DT_INST_PHANDLE(0, tcon));
	if (!device_is_ready(data->tcon)) {
		LOG_ERR("TCON not ready");
		return -ENODEV;
	}

	/* ---- 3. Clocks ---- */
	ret = clock_control_on(cfg->clock_dev, cfg->bus_clk);
	if (ret) {
		LOG_ERR("Bus clock: %d", ret);
		return ret;
	}
	ret = clock_control_on(cfg->clock_dev, cfg->mod_clk);
	if (ret) {
		LOG_ERR("Mod clock: %d", ret);
		clock_control_off(cfg->clock_dev, cfg->bus_clk);
		return ret;
	}

	/* ---- 4. Reset ---- */
	ret = reset_line_deassert_dt(&cfg->reset);
	if (ret) {
		LOG_ERR("Reset: %d", ret);
		goto err_clocks;
	}

	/* ---- 5. Clear + init blender ---- */
	de2_reset_clear(data->base);

	/*
	 * Enable RT_EN early (Linux does this FIRST in sun8i_mixer_init).
	 * Without RT_EN the double-buffered registers may not take effect.
	 */
	sys_write32(SUN8I_MIXER_GLOBAL_CTL_RT_EN,
		    data->base + SUN8I_MIXER_GLOBAL_CTL);

	de2_blender_init(data->base);

	/* ---- 6. Panel info ---- */
	data->width  = cfg->width;
	data->height = cfg->height;
	data->pixel_format = cfg->init_fmt;

	/* ---- 7. Allocate framebuffers ---- */
	/*
	 * Triple-buffering: one write buffer for the app, two display
	 * buffers for ping-pong with the DE2 DMA engine.
	 *
	 * The app writes to data->framebuffer at any time (returned by
	 * display_get_framebuffer).  On each de2_write(), the data is
	 * copied to the inactive display buffer, cache-flushed, and the
	 * DE2 layer address is switched to it.  This prevents tearing:
	 * the DE2 never reads from a buffer that the CPU is writing to.
	 */
	uint32_t bpp = DISPLAY_BITS_PER_PIXEL(data->pixel_format) / 8;
	data->fb_size = data->width * data->height * bpp;

	data->framebuffer = k_aligned_alloc(64, data->fb_size);
	if (!data->framebuffer) {
		LOG_ERR("Write FB alloc failed (%u bytes)", data->fb_size);
		ret = -ENOMEM;
		goto err_reset;
	}
	memset(data->framebuffer, 0x00, data->fb_size);

	data->fb_display[0] = k_aligned_alloc(64, data->fb_size);
	if (!data->fb_display[0]) {
		LOG_ERR("Display FB[0] alloc failed");
		ret = -ENOMEM;
		goto err_free_fb;
	}
	memset(data->fb_display[0], 0x00, data->fb_size);

	data->fb_display[1] = k_aligned_alloc(64, data->fb_size);
	if (!data->fb_display[1]) {
		LOG_ERR("Display FB[1] alloc failed");
		ret = -ENOMEM;
		goto err_free_disp0;
	}
	memset(data->fb_display[1], 0x00, data->fb_size);

	data->fb_active = 0;
	k_mutex_init(&data->lock);

	LOG_DBG("DE2 Mixer ready: %ux%u bpp=%u fb=%p disp[0]=%p disp[1]=%p base=0x%lx",
		data->width, data->height, bpp,
		data->framebuffer, data->fb_display[0], data->fb_display[1],
		(unsigned long)data->base);
	return 0;

err_free_disp0:
	k_free(data->fb_display[0]);
err_free_fb:
	k_free(data->framebuffer);
err_reset:
	reset_line_assert_dt(&cfg->reset);
err_clocks:
	clock_control_off(cfg->clock_dev, cfg->mod_clk);
	clock_control_off(cfg->clock_dev, cfg->bus_clk);
	return ret;
}

#define DE2_MIXER_INIT(inst)                                                   \
	static struct de2_mixer_data de2_mixer_data_##inst;                    \
                                                                                \
	static const struct de2_mixer_config de2_mixer_config_##inst = {        \
		.base      = DT_INST_REG_ADDR(inst),                             \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),           \
		.bus_clk   = (clock_control_subsys_t)(uintptr_t)                 \
			     DT_INST_CLOCKS_CELL_BY_NAME(inst, bus, clk_id),     \
		.mod_clk   = (clock_control_subsys_t)(uintptr_t)                 \
			     DT_INST_CLOCKS_CELL_BY_NAME(inst, mod, clk_id),     \
		.reset     = RESET_DT_SPEC_INST_GET(inst),                       \
		.width     = DT_INST_PROP(inst, width),                          \
		.height    = DT_INST_PROP(inst, height),                         \
		.init_fmt  = DT_INST_PROP(inst, pixel_format),                   \
	};                                                                      \
                                                                                \
	DEVICE_DT_INST_DEFINE(inst,                                             \
			      de2_mixer_init,                                   \
			      NULL,                                             \
			      &de2_mixer_data_##inst,                           \
			      &de2_mixer_config_##inst,                         \
			      POST_KERNEL,                                      \
			      CONFIG_DISPLAY_INIT_PRIORITY,                     \
			      &de2_mixer_api);

DT_INST_FOREACH_STATUS_OKAY(DE2_MIXER_INIT)
