/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_LOGGING_LOG_BACKEND_RAM_H_
#define ZEPHYR_INCLUDE_LOGGING_LOG_BACKEND_RAM_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct shell;

/**
 * @brief Display buffered log messages from the RAM backend.
 *
 * Prints formatted log messages stored in the RAM ring buffer to the
 * shell. Messages are displayed in chronological order (oldest first).
 *
 * @param sh        Shell instance for output.
 * @param max_lines Maximum number of lines to display. Use SIZE_MAX for all.
 */
void log_backend_ram_show(const struct shell *sh, size_t max_lines);

/**
 * @brief Clear the RAM log buffer.
 *
 * Resets the ring buffer, discarding all stored log messages.
 */
void log_backend_ram_clean(void);

/**
 * @brief Check if the RAM log backend is active.
 *
 * @return true if the backend is active and storing logs, false otherwise.
 */
bool log_backend_ram_is_active(void);

/**
 * @brief Get the current number of bytes stored in the RAM log buffer.
 *
 * @return Number of bytes currently buffered.
 */
uint32_t log_backend_ram_buffered_size(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_LOGGING_LOG_BACKEND_RAM_H_ */
