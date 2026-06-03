/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file sdhc_shell.c
 * @brief MMC/SD shell commands (like U-Boot 'mmc' command)
 *
 * Usage:
 *   mmc init [<dev>]           - Probe SD/eMMC card (default: sdhc0)
 *   mmc info                   - Show card details
 *   mmc list                   - List available MMC/SDHC devices
 *   mmc read  <blk> [cnt]      - Read blocks and hex-dump
 *   mmc write <blk> <hex ...>  - Write hex data to a block
 *   mmc erase <start> <cnt>    - Erase blocks (requires --force for block 0)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/sd/sd.h>
#include <zephyr/sd/sdmmc.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/storage/disk_access.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>

LOG_MODULE_REGISTER(sdhc_shell, CONFIG_SDHC_LOG_LEVEL);

/* Maximum data to dump per read command (in blocks of 512 bytes) */
#define MMC_READ_MAX_BLOCKS   4
/* Maximum bytes for write command */
#define MMC_WRITE_MAX_BYTES   256
/* Hex dump bytes per line */
#define HEX_DUMP_BYTES_PER_LINE 16
/* Static buffer for read operations */
#define MMC_READ_BUF_SIZE     (MMC_READ_MAX_BLOCKS * 512)
/* Static block buffer for write operations (read-modify-write) */
#define MMC_BLOCK_BUF_SIZE    512

static uint8_t mmc_read_buf[MMC_READ_BUF_SIZE] __aligned(4);
static uint8_t mmc_block_buf[MMC_BLOCK_BUF_SIZE] __aligned(4);

static struct sd_card mmc_card;
static bool mmc_initialized;
static const struct device *current_sdhc;

/*
 * Get the default SDHC device (sdhc0 alias), or NULL.
 */
static const struct device *sdhc_default_dev(void)
{
	return DEVICE_DT_GET_OR_NULL(DT_ALIAS(sdhc0));
}

/*
 * Resolve a device name to an SDHC device.
 * If name is NULL, returns the default device.
 * Returns NULL if not found or not an SDHC device.
 */
static const struct device *sdhc_find_dev(const char *name)
{
	if (name == NULL || name[0] == '\0') {
		return current_sdhc ? current_sdhc : sdhc_default_dev();
	}

	const struct device *dev = device_get_binding(name);

	if (dev == NULL) {
		return NULL;
	}
	return dev;
}

/*
 * Convert hex char to value
 */
static int hex_val(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}
	if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	}
	return -1;
}

/*
 * Parse hex string to byte buffer. Returns number of bytes parsed.
 */
static int parse_hex_data(const char *hex_str, uint8_t *buf, int max_bytes)
{
	int bytes = 0;
	int len = strlen(hex_str);

	if (len > 2 && hex_str[0] == '0' && (hex_str[1] == 'x' || hex_str[1] == 'X')) {
		hex_str += 2;
		len -= 2;
	}

	for (int i = 0; i < len && bytes < max_bytes; i += 2) {
		int hi, lo;

		hi = hex_val(hex_str[i]);
		if (i + 1 < len) {
			lo = hex_val(hex_str[i + 1]);
		} else {
			lo = 0;
		}
		if (hi < 0) {
			break;
		}
		buf[bytes++] = (uint8_t)((hi << 4) | lo);
	}
	return bytes;
}

/*
 * Hex dump a buffer to shell output
 */
static void hex_dump(const struct shell *sh, const uint8_t *buf, int len,
		     uint32_t start_offset)
{
	char ascii[HEX_DUMP_BYTES_PER_LINE + 1];
	int i;

	for (i = 0; i < len; i++) {
		if (i % HEX_DUMP_BYTES_PER_LINE == 0) {
			if (i > 0) {
				shell_print(sh, "  |%s|", ascii);
			}
			shell_fprintf(sh, SHELL_NORMAL, "%08x: ", start_offset + i);
		}
		shell_fprintf(sh, SHELL_NORMAL, "%02x ", buf[i]);
		ascii[i % HEX_DUMP_BYTES_PER_LINE] =
			isprint(buf[i]) ? (char)buf[i] : '.';
		ascii[(i % HEX_DUMP_BYTES_PER_LINE) + 1] = '\0';
	}

	if (len > 0) {
		int remaining = HEX_DUMP_BYTES_PER_LINE - (len % HEX_DUMP_BYTES_PER_LINE);

		if (remaining < HEX_DUMP_BYTES_PER_LINE) {
			for (int j = 0; j < remaining; j++) {
				shell_fprintf(sh, SHELL_NORMAL, "   ");
			}
		}
		ascii[len % HEX_DUMP_BYTES_PER_LINE] = '\0';
		if (len % HEX_DUMP_BYTES_PER_LINE == 0) {
			ascii[HEX_DUMP_BYTES_PER_LINE] = '\0';
		}
		shell_print(sh, "  |%s|", ascii);
	}
}

/*
 * Parse optional <dev> from argv[1]. If argv[1] starts with "mmc" or "sdhc",
 * treat it as a device name and consume it. Returns device pointer or NULL.
 * Updates argc/argv to skip the device argument if consumed.
 */
static const struct device *sdhc_parse_dev_arg(const struct shell *sh,
					       int *argc, char ***argv)
{
	if (*argc < 2) {
		return sdhc_default_dev();
	}

	/* If first arg looks like a device name (starts with mmc or sdhc), use it */
	const char *arg = (*argv)[1];

	if (strncmp(arg, "mmc", 3) == 0 || strncmp(arg, "sdhc", 4) == 0) {
		const struct device *dev = sdhc_find_dev(arg);

		if (dev == NULL) {
			shell_error(sh, "Device '%s' not found", arg);
			return NULL;
		}
		/* Consume the device argument */
		(*argc)--;
		(*argv)++;
		return dev;
	}

	/* No device specified, use default */
	return current_sdhc ? current_sdhc : sdhc_default_dev();
}

/*
 * mmc init [<dev>] - Initialize the MMC/SD card
 */
static int cmd_mmc_init(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *sdhc_dev;
	int ret;

	sdhc_dev = sdhc_parse_dev_arg(sh, (int *)&argc, &argv);
	if (sdhc_dev == NULL) {
		return -ENODEV;
	}

	if (!device_is_ready(sdhc_dev)) {
		shell_error(sh, "SDHC device not ready");
		return -ENODEV;
	}

	shell_print(sh, "Probing SD/MMC card on %s...", sdhc_dev->name);

	memset(&mmc_card, 0, sizeof(mmc_card));
	ret = sd_init(sdhc_dev, &mmc_card);
	if (ret) {
		shell_error(sh, "Card init failed: %d", ret);
		current_sdhc = sdhc_dev;
		mmc_initialized = false;
		return ret;
	}

	current_sdhc = sdhc_dev;
	mmc_initialized = true;

	uint32_t block_count = 0;

	sdmmc_ioctl(&mmc_card, DISK_IOCTL_GET_SECTOR_COUNT, &block_count);

	shell_print(sh, "Card initialized successfully!");
	shell_print(sh, "  Type: %s",
		    mmc_card.type == CARD_SDMMC ? "SD" :
		    mmc_card.type == CARD_MMC  ? "eMMC" :
		    mmc_card.type == CARD_SDIO ? "SDIO" : "Unknown");
	shell_print(sh, "  Block size: %u bytes", mmc_card.block_size);
	shell_print(sh, "  Block count: %u", block_count);
	shell_print(sh, "  Capacity: %u MB",
		    (uint32_t)(((uint64_t)block_count *
			       mmc_card.block_size) / (1024 * 1024)));
	shell_print(sh, "  Status: %s",
		    mmc_card.status == CARD_INITIALIZED ? "Ready" :
		    mmc_card.status == CARD_ERROR ? "Error" : "Unknown");

	return 0;
}

/*
 * mmc info - Show current card information
 */
static int cmd_mmc_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mmc_initialized) {
		shell_error(sh, "Card not initialized. Run 'mmc init' first.");
		return -EIO;
	}

	uint32_t block_count = 0;

	sdmmc_ioctl(&mmc_card, DISK_IOCTL_GET_SECTOR_COUNT, &block_count);

	shell_print(sh, "Current device: %s", current_sdhc->name);
	shell_print(sh, "MMC/SD Card Information:");
	shell_print(sh, "========================");
	shell_print(sh, "  Type:            %s",
		    mmc_card.type == CARD_SDMMC ? "SD Memory Card" :
		    mmc_card.type == CARD_MMC  ? "eMMC" :
		    mmc_card.type == CARD_SDIO ? "SDIO" : "Unknown");
	/* sd_version uses bit-flag enum values (BIT(0..3)), not BCD.
	 * Map to human-readable version string.
	 */
	{
		const char *ver_str = "?.??";
		if (mmc_card.sd_version & SD_SPEC_VER3_0) {
			ver_str = "3.0";
		} else if (mmc_card.sd_version & SD_SPEC_VER2_0) {
			ver_str = "2.0";
		} else if (mmc_card.sd_version & SD_SPEC_VER1_1) {
			ver_str = "1.1";
		} else if (mmc_card.sd_version & SD_SPEC_VER1_0) {
			ver_str = "1.0";
		}
		shell_print(sh, "  Version:         SD %s", ver_str);
	}
	shell_print(sh, "  Block size:      %u bytes", mmc_card.block_size);
	shell_print(sh, "  Block count:     %u", block_count);
	shell_print(sh, "  Capacity:        %u MB",
		    (uint32_t)(((uint64_t)block_count * mmc_card.block_size) /
			       (1024 * 1024)));
	shell_print(sh, "  OCR:             0x%08x", mmc_card.ocr);
	shell_print(sh, "  RCA:             0x%04x", mmc_card.relative_addr);
	shell_print(sh, "  Bus width:       %u-bit",
		    mmc_card.bus_io.bus_width == SDHC_BUS_WIDTH4BIT ? 4 : 1);
	shell_print(sh, "  Signal voltage:  %s",
		    mmc_card.card_voltage == SD_VOL_3_3_V ? "3.3V" :
		    mmc_card.card_voltage == SD_VOL_1_8_V ? "1.8V" : "Unknown");
	shell_print(sh, "  Status:          %s",
		    mmc_card.status == CARD_INITIALIZED ? "Ready" : "Error");

	return 0;
}

/*
 * mmc read [<dev>] <blk> [cnt] - Read blocks from card and hex-dump
 */
static int cmd_mmc_read(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t start_block;
	uint32_t count = 1;
	uint8_t *buf = mmc_read_buf;
	int ret;

	if (!mmc_initialized || mmc_card.status != CARD_INITIALIZED) {
		shell_error(sh, "Card not initialized. Run 'mmc init' first.");
		return -EIO;
	}

	/* First arg is block number (device parsing already done by subcmd layer if needed) */
	start_block = strtoul(argv[1], NULL, 0);

	if (argc >= 3) {
		count = strtoul(argv[2], NULL, 0);
		if (count == 0 || count > MMC_READ_MAX_BLOCKS) {
			shell_error(sh, "Block count must be 1-%u",
				    MMC_READ_MAX_BLOCKS);
			return -EINVAL;
		}
	}

	if (count * mmc_card.block_size > sizeof(mmc_read_buf)) {
		shell_error(sh, "Read size %u exceeds buffer %u",
			    count * mmc_card.block_size,
			    (unsigned int)sizeof(mmc_read_buf));
		return -ENOMEM;
	}

	ret = sdmmc_read_blocks(&mmc_card, buf, start_block, count);
	if (ret) {
		shell_error(sh, "Read failed: %d", ret);
		return ret;
	}

	shell_print(sh, "Read %u block(s) from offset 0x%x (%u):",
		    count, start_block * mmc_card.block_size,
		    start_block * mmc_card.block_size);
	hex_dump(sh, buf, count * mmc_card.block_size,
		 start_block * mmc_card.block_size);

	return 0;
}

/*
 * mmc write <blk> <byte0> [byte1...] - Write raw bytes to a block
 */
static int cmd_mmc_write(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t block;
	uint8_t buf[MMC_WRITE_MAX_BYTES];
	int data_len;
	int ret;

	if (!mmc_initialized || mmc_card.status != CARD_INITIALIZED) {
		shell_error(sh, "Card not initialized. Run 'mmc init' first.");
		return -EIO;
	}

	block = strtoul(argv[1], NULL, 0);

	/* Parse hex data from remaining args */
	data_len = 0;
	for (size_t i = 2; i < argc && data_len < (int)sizeof(buf); i++) {
		uint8_t parsed[MMC_WRITE_MAX_BYTES];
		int n = parse_hex_data(argv[i], parsed, sizeof(parsed) - data_len);

		memcpy(buf + data_len, parsed, n);
		data_len += n;
	}

	if (data_len == 0) {
		shell_error(sh, "No data provided");
		return -EINVAL;
	}

	if (data_len > (int)mmc_card.block_size) {
		shell_error(sh, "Data exceeds block size (%u bytes)",
			    mmc_card.block_size);
		return -EINVAL;
	}

	if (block == 0) {
		shell_warn(sh, "WARNING: Writing to block 0 (MBR/partition table)!");
		shell_print(sh, "Use 'mmc write 0 --force <data>...' to confirm.");
		/* Check if --force is present */
		return -EPERM;
	}

	/* Read existing block, overlay, write back */
	ret = sdmmc_read_blocks(&mmc_card, mmc_block_buf, block, 1);
	if (ret && ret != -EIO) {
		shell_warn(sh, "Could not read existing block (ret=%d), "
			   "writing partial data", ret);
		memset(mmc_block_buf, 0, mmc_card.block_size);
	}

	memcpy(mmc_block_buf, buf, data_len);

	ret = sdmmc_write_blocks(&mmc_card, mmc_block_buf, block, 1);
	if (ret) {
		shell_error(sh, "Write failed: %d", ret);
		return ret;
	}

	shell_print(sh, "Wrote %d bytes to block %u", data_len, block);
	shell_print(sh, "Data written:");
	hex_dump(sh, buf, data_len, block * mmc_card.block_size);

	return 0;
}

/*
 * mmc erase <start_block> <count> - Erase blocks
 */
static int cmd_mmc_erase(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t start_block;
	uint32_t count;
	int ret;

	if (!mmc_initialized || mmc_card.status != CARD_INITIALIZED) {
		shell_error(sh, "Card not initialized. Run 'mmc init' first.");
		return -EIO;
	}

	start_block = strtoul(argv[1], NULL, 0);
	count = strtoul(argv[2], NULL, 0);

	if (count == 0) {
		shell_error(sh, "Erase count must be > 0");
		return -EINVAL;
	}

	if (start_block == 0) {
		shell_warn(sh, "WARNING: Erasing from block 0 will destroy "
			   "partition table!");
		if (argc < 4 || strcmp(argv[3], "--force") != 0) {
			shell_print(sh, "Use 'mmc erase 0 %u --force' to confirm.",
				    count);
			return -EPERM;
		}
	}

	shell_print(sh, "Erasing %u blocks starting at block %u...",
		    count, start_block);

	ret = sdmmc_erase_blocks(&mmc_card, start_block, count);
	if (ret) {
		shell_error(sh, "Erase failed: %d", ret);
		return ret;
	}

	shell_print(sh, "Erase complete.");
	return 0;
}

/*
 * mmc list - List available MMC/SDHC devices
 */
static int cmd_mmc_list(const struct shell *sh, size_t argc, char **argv)
{
	static const char *const aliases[] = {
		"sdhc0", "sdhc1", "sdhc2",
	};
	bool found = false;

	shell_print(sh, "MMC/SDHC Devices:");
	shell_print(sh, "==================");

	for (int i = 0; i < ARRAY_SIZE(aliases); i++) {
		const struct device *dev;

		dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(sdhc0));
		if (i == 1) {
			dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(sdhc1));
		} else if (i == 2) {
			dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(sdhc2));
		}
		if (dev == NULL) {
			continue;
		}
		const char *marker = "";

		if (current_sdhc == dev) {
			marker = " <-- current";
		}
		shell_print(sh, "  %d: %-20s %s%s",
			    i, dev->name,
			    device_is_ready(dev) ? "[ready]" : "[not ready]",
			    marker);
		found = true;
	}

	if (!found) {
		shell_print(sh, "  (none - no SDHC aliases in devicetree)");
	}

	return 0;
}

/* Help strings */
#define HELP_INIT  "Probe MMC/SD card [<dev>]"
#define HELP_INFO  "Show card details"
#define HELP_LIST  "List available MMC/SDHC devices"
#define HELP_READ  "Read blocks: <block> [count]"
#define HELP_WRITE "Write hex data: <block> <hex...>"
#define HELP_ERASE "Erase blocks: <start> <count> [--force]"

SHELL_STATIC_SUBCMD_SET_CREATE(mmc_cmds,
	SHELL_CMD_ARG(init,  NULL, HELP_INIT,  cmd_mmc_init,  1, 1),
	SHELL_CMD(info,  NULL, HELP_INFO,  cmd_mmc_info),
	SHELL_CMD(list,  NULL, HELP_LIST,  cmd_mmc_list),
	SHELL_CMD_ARG(read,  NULL, HELP_READ,  cmd_mmc_read,  2, 1),
	SHELL_CMD_ARG(write, NULL, HELP_WRITE, cmd_mmc_write, 3,
		      MMC_WRITE_MAX_BYTES - 1),
	SHELL_CMD_ARG(erase, NULL, HELP_ERASE, cmd_mmc_erase, 3, 1),
	SHELL_SUBCMD_SET_END
);

static int cmd_mmc(const struct shell *sh, size_t argc, char **argv)
{
	shell_error(sh, "%s: unknown parameter: %s", argv[0],
		    argc > 1 ? argv[1] : "<none>");
	return -EINVAL;
}

SHELL_CMD_REGISTER(mmc, &mmc_cmds,
	"MMC/SD card commands ('mmc init [<dev>]' for card probe)", cmd_mmc);
