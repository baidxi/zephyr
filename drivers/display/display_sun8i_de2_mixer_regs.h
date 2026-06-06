/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner DE2 Mixer register definitions.
 *
 * Based on Linux sun8i_mixer.h.
 */

#ifndef ZEPHYR_DRIVERS_DISPLAY_SUN8I_DE2_MIXER_REGS_H_
#define ZEPHYR_DRIVERS_DISPLAY_SUN8I_DE2_MIXER_REGS_H_

/* Global control */
#define SUN8I_MIXER_GLOBAL_CTL          0x0000
#define SUN8I_MIXER_GLOBAL_CTL_RT_EN    BIT(0)
#define SUN8I_MIXER_GLOBAL_STATUS       0x0004
#define SUN8I_MIXER_GLOBAL_DBUFF        0x0008
#define SUN8I_MIXER_GLOBAL_DBUFF_ENABLE BIT(0)
#define SUN8I_MIXER_GLOBAL_SIZE         0x000C
#define SUN8I_MIXER_SIZE(w, h)          (((h) - 1) << 16 | ((w) - 1))

/* Blender (0x1000) */
#define DE2_BLD_BASE                    0x1000
#define DE2_CH_BASE                     0x2000
#define DE2_CH_SIZE                     0x1000

#define BLEND_PIPE_CTL(base)            ((base) + 0x00)
#define BLEND_PIPE_CTL_EN(pipe)         BIT(8 + pipe)
#define BLEND_PIPE_CTL_FC_EN(pipe)      BIT(pipe)
#define BLEND_ATTR_FCOLOR(base, x)      ((base) + 0x04 + 0x10 * (x))
#define BLEND_ATTR_INSIZE(base, x)      ((base) + 0x08 + 0x10 * (x))
#define BLEND_ATTR_COORD(base, x)       ((base) + 0x0C + 0x10 * (x))
#define BLEND_ROUTE(base)               ((base) + 0x80)
#define BLEND_PREMULTIPLY(base)         ((base) + 0x84)
#define BLEND_BKCOLOR(base)             ((base) + 0x88)
#define BLEND_OUTSIZE(base)             ((base) + 0x8C)
#define BLEND_MODE(base, x)             ((base) + 0x90 + 0x04 * (x))
#define BLEND_CK_CTL(base)              ((base) + 0xB0)
#define BLEND_CK_CFG(base)              ((base) + 0xB4)
#define BLEND_OUTCTL(base)              ((base) + 0xFC)
#define BLEND_OUTCTL_INTERLACED         BIT(1)

#define BLEND_MODE_DEF                  0x03010301

/* UI layer registers (per channel, per overlay) */
/* V3s: vi_num=2, ui_num=1 → UI layer = channel[2], overlay[0] */
#define CHAN_UI_LAYER_ATTR(ch, ovl)     ((ch) + 0x00 + 0x80 * (ovl))
#define   LAYER_ATTR_EN                 BIT(0)
#define   LAYER_ATTR_ALPHA_MODE_MASK    GENMASK(2, 1)
#define   LAYER_ATTR_ALPHA_MODE_PIXEL   (0 << 1)
#define   LAYER_ATTR_ALPHA_MODE_LAYER   (1 << 1)
#define   LAYER_ATTR_ALPHA_MODE_COMBINED (2 << 1)
#define   LAYER_ATTR_ALPHA_SHIFT        24
#define   LAYER_ATTR_ALPHA(x)           ((x) << 24)
#define   LAYER_ATTR_FBFMT_SHIFT        8

#define CHAN_UI_LAYER_SIZE(ch, ovl)     ((ch) + 0x04 + 0x80 * (ovl))
#define CHAN_UI_LAYER_COORD(ch, ovl)    ((ch) + 0x08 + 0x80 * (ovl))
#define CHAN_UI_LAYER_PITCH(ch, ovl)    ((ch) + 0x0C + 0x80 * (ovl))
#define CHAN_UI_LAYER_TOP_LADDR(ch, ovl) ((ch) + 0x10 + 0x80 * (ovl))
#define CHAN_UI_OVL_SIZE(ch)            ((ch) + 0x88)

/* DE2 pixel format codes (subset used in Zephyr) */
#define DE2_FBFMT_ARGB8888              0
#define DE2_FBFMT_XRGB8888              4
#define DE2_FBFMT_RGB888                8
#define DE2_FBFMT_RGB565                10

/* Unused sub-engines to disable */
#define SUN8I_MIXER_FCE_EN              0xA0000
#define SUN8I_MIXER_BWS_EN              0xA2000
#define SUN8I_MIXER_LTI_EN              0xA4000
#define SUN8I_MIXER_PEAK_EN             0xA6000
#define SUN8I_MIXER_ASE_EN              0xA8000
#define SUN8I_MIXER_FCC_EN              0xAA000
#define SUN8I_MIXER_DCSC_EN             0xB0000

#define DE2_MIXER_UNIT_SIZE             0x6000

/* UI channel index on V3s (vi_num=2, so UI = channel 2) */
#define MIXER_UI_CHANNEL                2
#define MIXER_UI_OVERLAY                0

#endif /* ZEPHYR_DRIVERS_DISPLAY_SUN8I_DE2_MIXER_REGS_H_ */
