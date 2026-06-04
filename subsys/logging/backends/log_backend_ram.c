/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_backend_std.h>
#include <zephyr/logging/log_backend_ram.h>
#include <zephyr/logging/log_core.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>

/* The ring buffer that stores formatted log text. Uses ring_buf_init
 * at runtime since the size comes from Kconfig (not necessarily a
 * compile-time constant for the RING_BUF_DECLARE assert).
 */
static uint8_t ramlog_data[CONFIG_LOG_BACKEND_RAM_BUF_SIZE];
static struct ring_buf ramlog_ringbuf;

/* Saved pointer to our own backend instance, set during init(). */
static const struct log_backend *self_backend;

/* Mutex to protect the ring buffer from concurrent access by the
 * logging thread (writer) and the shell thread (reader).
 */
static K_MUTEX_DEFINE(ramlog_mutex);

/* Log output format: text, sys-t, dictionary, or custom. */
static uint32_t log_format_current = LOG_OUTPUT_TEXT;

/* Internal scratch buffer for log_output formatting. */
static uint8_t scratch_buf[CONFIG_LOG_BACKEND_RAM_OUTPUT_BUF_SIZE];

/*---------------------------------------------------------------------
 * log_output_func_t callback: receives formatted log chunks and
 * appends them to the ring buffer.
 *-------------------------------------------------------------------*/
static int ramlog_out(uint8_t *data, size_t length, void *ctx)
{
	ARG_UNUSED(ctx);

	k_mutex_lock(&ramlog_mutex, K_FOREVER);

	/* If there isn't enough free space, discard oldest data first
	 * (overwrite / circular buffer semantics).
	 */
	uint32_t space = ring_buf_space_get(&ramlog_ringbuf);

	if (space < length) {
		uint32_t shortfall = length - space;
		uint8_t discard[64];
		uint32_t discarded = 0;

		while (discarded < shortfall) {
			uint32_t n = ring_buf_get(&ramlog_ringbuf,
						  discard,
						  MIN(sizeof(discard),
						      shortfall - discarded));
			if (n == 0) {
				break;
			}
			discarded += n;
		}
	}

	uint32_t written = ring_buf_put(&ramlog_ringbuf, data, length);

	k_mutex_unlock(&ramlog_mutex);

	return (int)written;
}

LOG_OUTPUT_DEFINE(log_output_ram, ramlog_out,
		  scratch_buf, sizeof(scratch_buf));

/*---------------------------------------------------------------------
 * Backend API callbacks
 *-------------------------------------------------------------------*/

static void process(const struct log_backend *const backend,
		    union log_msg_generic *msg)
{
	uint32_t flags = log_backend_std_get_flags();
	log_format_func_t log_output_func =
		log_format_func_t_get(log_format_current);

	log_output_func(&log_output_ram, &msg->log, flags);
}

static void panic(struct log_backend const *const backend)
{
	log_output_flush(&log_output_ram);
}

static void dropped(const struct log_backend *const backend, uint32_t cnt)
{
	log_output_dropped_process(&log_output_ram, cnt);
}

static void init(const struct log_backend *const backend)
{
	self_backend = backend;
	ring_buf_init(&ramlog_ringbuf,
		      sizeof(ramlog_data),
		      ramlog_data);
}

static int format_set(const struct log_backend *const backend,
		      uint32_t log_type)
{
	log_format_current = log_type;
	return 0;
}

const struct log_backend_api log_backend_ram_api = {
	.process = process,
	.panic = panic,
	.dropped = IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE) ? NULL : dropped,
	.init = init,
	.format_set = format_set,
};

LOG_BACKEND_DEFINE(log_backend_ram, log_backend_ram_api,
		   IS_ENABLED(CONFIG_LOG_BACKEND_RAM_AUTOSTART));

/*---------------------------------------------------------------------
 * Public API for shell commands
 *-------------------------------------------------------------------*/

void log_backend_ram_show(const struct shell *sh, size_t max_lines)
{
	k_mutex_lock(&ramlog_mutex, K_FOREVER);

	uint32_t total = ring_buf_size_get(&ramlog_ringbuf);

	if (total == 0) {
		shell_print(sh, "(RAM log buffer is empty)");
		k_mutex_unlock(&ramlog_mutex);
		return;
	}

	/* Allocate a temporary buffer to hold all peeking data.
	 * Use k_malloc for potentially large buffers.
	 */
	uint8_t *tmp = k_malloc(total);

	if (!tmp) {
		shell_error(sh,
			    "Not enough memory to display log buffer "
			    "(%u bytes needed)", total);
		k_mutex_unlock(&ramlog_mutex);
		return;
	}

	/* Copy out all data (ring_buf_get handles wrap-around internally). */
	uint32_t n = ring_buf_get(&ramlog_ringbuf, tmp, total);

	/* Immediately restore the data so the ring buffer is unchanged. */
	ring_buf_reset(&ramlog_ringbuf);
	ring_buf_put(&ramlog_ringbuf, tmp, n);

	/* Parse and display the buffered text line by line. */
	size_t lines = 0;
	char line[256];
	size_t lp = 0;

	for (uint32_t i = 0; i < n && lines < max_lines; i++) {
		if (tmp[i] == '\n') {
			line[lp] = '\0';
			shell_fprintf(sh, SHELL_NORMAL, "%s\r\n", line);
			lp = 0;
			lines++;
		} else if (tmp[i] != '\r') {
			if (lp < sizeof(line) - 1) {
				line[lp++] = (char)tmp[i];
			}
		}
	}

	/* Handle a possible trailing line without newline terminator. */
	if (lp > 0 && lines < max_lines) {
		line[lp] = '\0';
		shell_fprintf(sh, SHELL_NORMAL, "%s\r\n", line);
	}

	k_free(tmp);
	k_mutex_unlock(&ramlog_mutex);
}

void log_backend_ram_clean(void)
{
	k_mutex_lock(&ramlog_mutex, K_FOREVER);
	ring_buf_reset(&ramlog_ringbuf);
	k_mutex_unlock(&ramlog_mutex);
}

bool log_backend_ram_is_active(void)
{
	return (self_backend != NULL) &&
	       log_backend_is_active(self_backend);
}

uint32_t log_backend_ram_buffered_size(void)
{
	k_mutex_lock(&ramlog_mutex, K_FOREVER);
	uint32_t size = ring_buf_size_get(&ramlog_ringbuf);
	k_mutex_unlock(&ramlog_mutex);
	return size;
}
