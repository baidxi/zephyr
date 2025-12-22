#include <stdlib.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

LOG_MODULE_REGISTER(lszrz, LOG_LEVEL_INF);

/* ZMODEM protocol definitions */
#define ZPAD      '*'           /* 052 Padding character begins frames */
#define ZDLE      030           /* Ctrl-X Zmodem escape - `ala BISYNC DLE */
#define ZDLEE     (ZDLE ^ 0100) /* Escaped ZDLE as transmitted */
#define ZBIN      'A'           /* Binary frame indicator */
#define ZHEX      'B'           /* HEX frame indicator */
#define ZBIN32    'C'           /* Binary frame with 32 bit crc */
#define ZBINR32   'D'           /* RLE packed binary frame with 32 bit crc */
#define ZVBIN     'a'           /* binary frame indicator (crc16) */
#define ZVHEX     'b'           /* HEX frame indicator */
#define ZVBIN32   'c'           /* Binary frame indicator with 32 bit crc */
#define ZVBINR32  'd'           /* RLE packed Binary frame indicator with 32 bit crc */
#define ZRESC     0x7e          /* RLE flag/escape character */
#define ZMAXHLEN  16            /* Max header information length */
#define ZMAXSPLEN 1024          /* Max subpacket length */

/* Frame types */
#define ZRQINIT    0
#define ZRINIT     1
#define ZSINIT     2
#define ZACK       3
#define ZFILE      4
#define ZSKIP      5
#define ZNAK       6
#define ZABORT     7
#define ZFIN       8
#define ZRPOS      9
#define ZDATA      10
#define ZEOF       11
#define ZFERR      12
#define ZCRC       13
#define ZCHALLENGE 14
#define ZCOMPL     15
#define ZCAN       16
#define ZFREECNT   17
#define ZCOMMAND   18
#define ZSTDERR    19

/* ZMODEM state */
enum zm_state {
	ZM_STATE_IDLE,
	ZM_STATE_INIT,
	ZM_STATE_HEADER,
	ZM_STATE_DATA,
	ZM_STATE_EOF,
	ZM_STATE_FIN,
	ZM_STATE_ERROR
};

/* File transfer context */
#define MAX_FILE_SIZE (1024 * 1024) /* 1MB max file size */

enum {
	TYPE_RAM,
	TYPE_FLASH,
};

#define STACK_SIZE 2048

struct zm_ctx {
	enum zm_state state;
	char filename[256];
	size_t file_size;
	size_t bytes_received;
	uint8_t *file_buffer; /* Memory buffer for file data */
	uint8_t rx_buffer[1024];
	const struct shell *shell;
	uint32_t type;
	uint8_t init_buffer[3]; /* Buffer for "rz\r" sequence */
	size_t init_len;
	struct k_thread thread;
	K_KERNEL_STACK_MEMBER(stack, STACK_SIZE);
	k_tid_t tid;
	struct k_msgq msgq;
	struct ring_buf rx_ring;
	uint8_t zm_type;
	size_t rx_pos; /* Position in rx_buffer */
	uint8_t buf[16]; /* Buffer for frame processing */
	size_t buf_len; /* Length of data in buf */
};

/* Forward declarations */
static uint16_t crc16_ccitt(const uint8_t *buf, int len);

/* Send a byte to the serial port */
static void zm_send_byte(struct zm_ctx *ctx, uint8_t c)
{
	if (ctx->shell) {
		shell_print(ctx->shell, "%c", c);
	}
}

/* Send a buffer to the serial port */
static void zm_send_buffer(struct zm_ctx *ctx, const uint8_t *buf, size_t len)
{
	if (ctx->shell) {
		// z_shell_write(ctx->shell, buf, len);
		for (size_t i = 0; i < len; i++) {
			shell_print(ctx->shell, "%c", buf[i]);
		}
	}
}

/* Send ZMODEM hex header */
static void zm_send_hex_header(struct zm_ctx *ctx, uint8_t type, uint8_t *data)
{
	uint8_t header[21] = {0};
	uint16_t crc;

	header[0] = ZPAD;
	header[1] = ZPAD;
	header[2] = ZDLE;
	header[3] = ZHEX;
	header[4] = type;

	for (int i = 0; i < 4; i++) {
		uint8_t val = data[i];
		header[5 + i * 2] = "0123456789abcdef"[val >> 4];
		header[5 + i * 2 + 1] = "0123456789abcdef"[val & 0x0F];
	}

	crc = crc16_ccitt(&header[4], 5);
	header[13] = "0123456789abcdef"[crc >> 12];
	header[14] = "0123456789abcdef"[(crc >> 8) & 0x0F];
	header[15] = "0123456789abcdef"[(crc >> 4) & 0x0F];
	header[16] = "0123456789abcdef"[crc & 0x0F];

	header[17] = '\r';
	header[18] = '\n';
	if (type != ZACK && type != ZFIN) {
		header[19] = 0x8A; /* XON */
		zm_send_buffer(ctx, header, 20);
	} else {
		zm_send_buffer(ctx, header, 19);
	}
}

/* Send ZMODEM response */
static void zm_send_response(struct zm_ctx *ctx, uint8_t type, uint32_t pos)
{
	uint8_t header_data[4];

	header_data[0] = pos & 0xFF;
	header_data[1] = (pos >> 8) & 0xFF;
	header_data[2] = (pos >> 16) & 0xFF;
	header_data[3] = (pos >> 24) & 0xFF;

	zm_send_hex_header(ctx, type, header_data);
	LOG_INF("Sent ZMODEM response: type=%d, pos=%d", type, pos);
}

static void zm_handle_error(struct zm_ctx *ctx, const char *message)
{
	LOG_ERR("ZMODEM error: %s", message);
	ctx->state = ZM_STATE_ERROR;

	/* Send ZABORT to abort transfer */
	zm_send_response(ctx, ZABORT, 0);
}

static int zm_file_write(struct zm_ctx *ctx, const void *buf, size_t len)
{
	ctx->bytes_received += len;

	/* Log progress every 10KB */
	if (ctx->bytes_received % 10240 == 0) {
		LOG_INF("Received %d bytes (%d%%)", ctx->bytes_received,
			ctx->file_size > 0 ? (ctx->bytes_received * 100 / ctx->file_size) : 0);
	}

	return 0;
}

static int zm_file_open(struct zm_ctx *ctx, const char *filename)
{
	strncpy(ctx->filename, filename, sizeof(ctx->filename) - 1);
	ctx->filename[sizeof(ctx->filename) - 1] = '\0';
	ctx->bytes_received = 0;
	ctx->file_size = 0;

	LOG_INF("Preparing to receive file: %s", filename);
	return 0;
}

static void zm_file_close(struct zm_ctx *ctx)
{
	LOG_INF("File transfer complete: %s, received %d bytes", ctx->filename,
		ctx->bytes_received);
}

static uint16_t crc16_ccitt(const uint8_t *buf, int len)
{
	uint16_t crc = 0;

	while (len--) {
		crc = (crc << 8) ^ *buf++;
		for (int i = 0; i < 8; i++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

static int zm_get_hex_byte(const uint8_t **data, size_t *len)
{
	int c1, c2;

	if (*len < 2) {
		return -1;
	}

	c1 = (*data)[0];
	c2 = (*data)[1];
	*data += 2;
	*len -= 2;

	c1 = (c1 >= 'a') ? (c1 - 'a' + 10) : ((c1 >= '0') ? (c1 - '0') : c1);
	c2 = (c2 >= 'a') ? (c2 - 'a' + 10) : ((c2 >= '0') ? (c2 - '0') : c2);

	return (c1 << 4) | c2;
}

static int zm_get_hex_byte_from_data(const uint8_t *data, int offset)
{
	int c1, c2;

	c1 = data[offset];
	c2 = data[offset + 1];

	c1 = (c1 >= 'a') ? (c1 - 'a' + 10) : ((c1 >= '0') ? (c1 - '0') : c1);
	c2 = (c2 >= 'a') ? (c2 - 'a' + 10) : ((c2 >= '0') ? (c2 - '0') : c2);

	return (c1 << 4) | c2;
}

static int zm_read_hex_header(const uint8_t **data, size_t *len, uint8_t *header_data)
{
	uint8_t c;
	uint16_t crc_remote, crc_local;

	/* Find start of header */
	while (*len > 0) {
		c = *(*data)++;
		(*len)--;
		if (c == ZHEX) {
			break;
		}
	}

	if (*len < 12) { /* Minimum header size */
		return -1;
	}

	/* Skip ZPAD, ZDLE if present */
	if (*len > 0 && **data == ZPAD) {
		(*data)++;
		(*len)--;
	}
	if (*len > 0 && **data == ZDLE) {
		(*data)++;
		(*len)--;
	}
	if (*len > 0 && **data == ZHEX) {
		(*data)++;
		(*len)--;
	}

	uint8_t type = zm_get_hex_byte(data, len);
	if (type < 0) {
		return -1;
	}

	for (int i = 0; i < 4; i++) {
		int val = zm_get_hex_byte(data, len);
		if (val < 0) {
			return -1;
		}
		header_data[i] = val;
	}

	crc_remote = zm_get_hex_byte(data, len) << 8;
	crc_remote |= zm_get_hex_byte(data, len);

	uint8_t crc_buf[5] = {type, header_data[0], header_data[1], header_data[2], header_data[3]};
	crc_local = crc16_ccitt(crc_buf, 5);

	if (crc_remote != crc_local) {
		return -1;
	}

	return type;
}

static int zm_process_data(struct zm_ctx *ctx)
{
	size_t data_len = ring_buf_size_get(&ctx->rx_ring);
	uint8_t data[64] = {0};

	switch (ctx->state) {
		case ZM_STATE_IDLE:
		{
			if (data_len < 3)
				break;

			ring_buf_get(&ctx->rx_ring, data, 3);
			data_len -= 3;
			if (data[0] == 'r' && data[1] == 'z' && data[2] == '\r') {
				ctx->state = ZM_STATE_INIT;
				LOG_INF("Received 'rz' command, initializing ZMODEM");
			}
			break;
		}
		case ZM_STATE_INIT:
		{
			if (data_len < 4)
				break;

			ring_buf_get(&ctx->rx_ring, data, 4);
			data_len -= 4;
			if (data[0] != ZPAD)
			{
				break;
			}

			if (data[1] == ZPAD)
			{
				if (data[2] != ZDLE)
				{
					break;
				}
				ctx->zm_type = data[3];
			} else if (data[1] == ZDLE)
			{
				ctx->zm_type = data[2];
			} else {
				break;
			}

			/* Move to header state */
			ctx->state = ZM_STATE_HEADER;
			ctx->buf_len = 0;
			zm_send_response(ctx, ZRINIT, 0);
			LOG_INF("ZMODEM frame start detected, sent ZRINIT");
			break;
		}

		case ZM_STATE_HEADER: {
			int frame_type = 0;
			uint8_t header_data[4] = {0};
			const uint8_t *data_ptr = data;
			size_t data_size = 16;
			
			/* Get more data if needed */
			if (data_len < 16) {
				break;
			}
			
			/* Read data from ring buffer */
			ring_buf_get(&ctx->rx_ring, data, 16);
			data_len -= 16;
			
			/* Debug: Print the received data */
			LOG_INF("Received header data:");
			for (int i = 0; i < 16; i++) {
				LOG_INF("data[%d] = 0x%02x", i, data[i]);
			}
			
			/* Check if this is a hex header (14 '0's + 0x0d + 0x8a) */
			if (data[14] == 0x0d && data[15] == 0x8a) {
				/* This is likely a hex header, parse it directly */
				/* The first 14 bytes should be hex digits representing the frame type and data */
				
				/* Extract frame type from first two hex digits */
				frame_type = zm_get_hex_byte_from_data(data, 0);
				
				/* Extract 4 bytes of header data */
				for (int i = 0; i < 4; i++) {
					header_data[i] = zm_get_hex_byte_from_data(data, 2 + i * 2);
				}
				
				LOG_INF("Direct parsed frame_type: %d", frame_type);
			} else {
				/* Try to parse hex header using standard method */
				data_ptr = data;
				data_size = 16;
				frame_type = zm_read_hex_header(&data_ptr, &data_size, header_data);

				if (frame_type < 0) {
					zm_handle_error(ctx, "Invalid ZMODEM header");
					break;
				}
			}

			switch (frame_type) {
			case ZRQINIT:
				/* Sender wants to initiate ZMODEM session */
				LOG_INF("Received ZRQINIT, sending ZRINIT");
				zm_send_response(ctx, ZRINIT, 0);
				break;

			case ZFILE: {
				/* Extract filename and size from ZFILE data */
				size_t filename_len = 0;
				const uint8_t *filename_start = data_ptr;

				/* Find null terminator for filename */
				while (filename_len < data_size && data_ptr[filename_len] != '\0') {
					filename_len++;
				}

				if (filename_len < data_size && filename_len > 0) {
					char filename[256];
					strncpy(filename, (const char *)filename_start,
						MIN(filename_len, sizeof(filename) - 1));
					filename[MIN(filename_len, sizeof(filename) - 1)] = '\0';

					/* Skip filename and null terminator */
					data_ptr += filename_len + 1;
					data_size -= filename_len + 1;

					/* Extract file size */
					size_t size_len = 0;
					const uint8_t *size_start = data_ptr;

					/* Find null terminator for size */
					while (size_len < data_size && data_ptr[size_len] != '\0') {
						size_len++;
					}

					if (size_len < data_size && size_len > 0) {
						char size_str[32];
						strncpy(size_str, (const char *)size_start,
							MIN(size_len, sizeof(size_str) - 1));
						size_str[MIN(size_len, sizeof(size_str) - 1)] = '\0';

						ctx->file_size = atoi(size_str);

						/* Open file for writing */
						if (zm_file_open(ctx, filename) == 0) {
							LOG_INF("Receiving file: %s (%d bytes)",
								filename, ctx->file_size);
							/* Send ZRPOS to acknowledge file info */
							zm_send_response(ctx, ZRPOS, 0);
							ctx->state = ZM_STATE_DATA;
						} else {
							zm_handle_error(ctx, "Failed to open file");
						}
					}
				}
				break;
			}

			case ZDATA:
				ctx->state = ZM_STATE_DATA;
				ctx->rx_pos = 0;
				break;

			case ZEOF:
				zm_file_close(ctx);
				/* Send ZACK to acknowledge EOF */
				zm_send_response(ctx, ZACK, ctx->bytes_received);
				ctx->state = ZM_STATE_FIN;
				LOG_INF("File transfer complete");
				break;

			case ZFIN:
				/* Send ZFIN to acknowledge session end */
				zm_send_response(ctx, ZFIN, 0);
				ctx->state = ZM_STATE_FIN;
				LOG_INF("ZMODEM session finished");
				break;

			case ZNAK:
				zm_handle_error(ctx, "Received ZNAK - sender reports error");
				break;

			case ZABORT:
				zm_handle_error(ctx, "Received ZABORT - transfer aborted by sender");
				break;

			case ZCAN:
				zm_handle_error(ctx, "Received ZCAN - transfer cancelled");
				break;

			case ZFERR:
				zm_handle_error(ctx, "Received ZFERR - file I/O error on sender side");
				break;

			default:
				LOG_INF("Received ZMODEM frame type: %d", frame_type);
				break;
			}
			break;
		}

		case ZM_STATE_DATA:
			/* Process data until ZDLE sequence */
			while (data_len > 0) {
				uint8_t byte;
				ring_buf_get(&ctx->rx_ring, &byte, 1);
				data_len--;

				if (byte == ZDLE) {
					if (data_len == 0) {
						break;
					}

					/* Get next byte after ZDLE */
					ring_buf_get(&ctx->rx_ring, &byte, 1);
					data_len--;

					/* Handle ZDLE escape sequences */
					switch (byte) {
					case ZCRC:
						/* End of data frame */
						ctx->state = ZM_STATE_HEADER;
						/* Write any remaining data */
						if (ctx->rx_pos > 0) {
							zm_file_write(ctx, ctx->rx_buffer, ctx->rx_pos);
							ctx->rx_pos = 0;
						}
						break;

					default:
						/* ZDLE escape character */
						if ((byte >= 0x40 && byte < 0x60) ||
						    (byte >= 0x80 && byte < 0xa0)) {
							ctx->rx_buffer[ctx->rx_pos++] = byte ^ 0x40;
						} else {
							ctx->rx_buffer[ctx->rx_pos++] = byte;
						}
						break;
					}
				} else {
					ctx->rx_buffer[ctx->rx_pos++] = byte;
				}

				/* Write buffer if full */
				if (ctx->rx_pos >= sizeof(ctx->rx_buffer)) {
					zm_file_write(ctx, ctx->rx_buffer, ctx->rx_pos);
					ctx->rx_pos = 0;
				}
			}

			/* Write remaining data */
			if (ctx->rx_pos > 0) {
				zm_file_write(ctx, ctx->rx_buffer, ctx->rx_pos);
				ctx->rx_pos = 0;
			}
			break;

		case ZM_STATE_FIN:
			/* Wait for session to end */
			shell_set_bypass(ctx->shell, NULL, NULL);
			break;

		case ZM_STATE_ERROR:
			LOG_ERR("ZMODEM error state");
			shell_set_bypass(ctx->shell, NULL, NULL);
			break;

		default:
			break;
	}

	return 0;
}

static void zm_lrzsz_data_inject(const struct shell *shell, uint8_t *data, size_t len,
				 void *user_data)
{
	struct zm_ctx *ctx = user_data;

	ring_buf_put(&ctx->rx_ring, data, len);
}

static void zm_lrzsz_thread_func(void *v1, void *v2, void *v3)
{
	struct zm_ctx *ctx = v1;

	while (ctx->state != ZM_STATE_FIN && ctx->state != ZM_STATE_ERROR) {
		zm_process_data(ctx);
	}
}

static struct zm_ctx zm_ctx = {0};

static int cmd_rz(const struct shell *shell, size_t argc, char **argv)
{

	char *arg = argv[1];

	ring_buf_init(&zm_ctx.rx_ring, sizeof(zm_ctx.rx_buffer), zm_ctx.rx_buffer);

	zm_ctx.state = ZM_STATE_IDLE;
	zm_ctx.shell = shell;
	zm_ctx.rx_pos = 0;
	zm_ctx.buf_len = 0;

	if (strncmp(arg, "-t", 2) == 0) {
		arg = argv[2];
		if (strncmp(arg, "ram", 3) == 0) {
			zm_ctx.type = TYPE_RAM;
		} else if (strncmp(arg, "flash", 5) == 0) {
			zm_ctx.type = TYPE_FLASH;
		} else {
			shell_print(shell, "Unk type:%s\n", arg);
			return -EINVAL;
		}
	} else {
		shell_print(shell, "Unk argument:%s\n", arg);
		return -EINVAL;
	}

	if (zm_ctx.tid == 0) {
		zm_ctx.tid = k_thread_create(&zm_ctx.thread, zm_ctx.stack, STACK_SIZE,
					     zm_lrzsz_thread_func, &zm_ctx, NULL, NULL, 50, 0,
					     K_NO_WAIT);
	} else {
		k_thread_start(zm_ctx.tid);
	}

	shell_set_bypass(shell, zm_lrzsz_data_inject, &zm_ctx);

	return 0;
}

SHELL_CMD_ARG_REGISTER(rz, NULL, "rz recv file", cmd_rz, 3, 0);
