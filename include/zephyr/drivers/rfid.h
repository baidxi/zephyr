/**
 * @file
 * @brief Public API for RFID drivers
 */

/*
 * Copyright (c) 2025, juno
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RFID_H_
#define ZEPHYR_INCLUDE_DRIVERS_RFID_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief RFID driver API
 * @defgroup rfid_driver_api RFID Driver API
 * @ingroup io_interfaces
 * @{
 */

/**
 * @brief RFID trigger types.
 */
enum rfid_trigger {
	/** A card has entered the field. */
	RFID_TRIGGER_CARD_DETECTED,
};

/**
 * @typedef rfid_trigger_handler_t
 * @brief Callback API upon firing of a trigger.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param trigger The trigger type.
 */
typedef void (*rfid_trigger_handler_t)(const struct device *dev, enum rfid_trigger trigger);

/**
 * @brief Callback API for checking if a card is present.
 *
 * This sends a REQA or WUPA command.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param[out] atqa Pointer to a buffer to store the ATQA response (2 bytes).
 * @return 0 on success, negative error code on failure.
 */
typedef int (*rfid_api_is_present)(const struct device *dev, uint8_t *atqa);

/**
 * @brief Callback API for selecting a card and getting its UID.
 *
 * This performs the anticollision loop and selection process.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param[out] uid Pointer to a buffer to store the UID.
 * @param[in,out] uid_len As input, the size of the UID buffer. As output, the actual UID length.
 * @param[out] sak Pointer to a buffer to store the SAK (Select Acknowledge) value.
 * @return 0 on success, negative error code on failure.
 */
typedef int (*rfid_api_select)(const struct device *dev, uint8_t *uid, size_t *uid_len, uint8_t *sak);

/**
 * @brief Callback API to halt a card.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @return 0 on success, negative error code on failure.
 */
typedef int (*rfid_api_halt)(const struct device *dev);

/**
 * @brief Callback API for sending a RATS (Request for Answer to Select) command.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param[out] ats Pointer to a buffer to store the ATS (Answer to Select) response.
 * @param[in,out] ats_len As input, the size of the ATS buffer. As output, the actual ATS length.
 * @return 0 on success, negative error code on failure.
 */
typedef int (*rfid_api_rats)(const struct device *dev, uint8_t *ats, size_t *ats_len);

/**
 * @brief Callback API for sending a PPS (Protocol and Parameter Selection) command.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param dsi DSI (Divisor Send Integer) value for the PCD to PICC bitrate.
 * @param dri DRI (Divisor Receive Integer) value for the PICC to PCD bitrate.
 * @return 0 on success, negative error code on failure.
 */
typedef int (*rfid_api_pps)(const struct device *dev, uint8_t dsi, uint8_t dri);

/**
 * @brief Callback API for data exchange (transceive).
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param[in] tx_buf Pointer to the buffer containing data to transmit.
 * @param tx_len Length of the data to transmit.
 * @param[out] rx_buf Pointer to the buffer to store received data.
 * @param[in,out] rx_len As input, the size of the receive buffer. As output, the actual length of received data.
 * @return 0 on success, negative error code on failure.
 */
typedef int (*rfid_api_transceive)(const struct device *dev, const uint8_t *tx_buf, size_t tx_len,
				   uint8_t *rx_buf, size_t *rx_len);

/**
 * @brief Callback API for sending a WTX (Waiting Time eXtension) command.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param wtxm WTX multiplier.
 * @return 0 on success, negative error code on failure.
 */
typedef int (*rfid_api_wtx)(const struct device *dev, uint8_t wtxm);

/**
 * @brief Callback API for deselecting a card.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @return 0 on success, negative error code on failure.
 */
typedef int (*rfid_api_deselect)(const struct device *dev);

/**
 * @brief Callback API for setting a trigger.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param trig The trigger to set.
 * @param handler The function to call when the trigger fires.
 * @return 0 on success, negative error code on failure.
 */
typedef int (*rfid_api_trigger_set)(const struct device *dev, enum rfid_trigger trig,
				    rfid_trigger_handler_t handler);

/**
 * @brief RFID driver API structure.
 */
struct rfid_driver_api {
	rfid_api_is_present is_present;
	rfid_api_select select;
	rfid_api_halt halt;
	rfid_api_rats rats;
	rfid_api_pps pps;
	rfid_api_transceive transceive;
	rfid_api_wtx wtx;
	rfid_api_deselect deselect;
	rfid_api_trigger_set trigger_set;
};

/**
 * @brief Check if a card is present in the field.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param[out] atqa Pointer to a buffer to store the ATQA response (2 bytes).
 * @return 0 on success, negative error code on failure.
 */
static inline int rfid_is_present(const struct device *dev, uint8_t *atqa)
{
	const struct rfid_driver_api *api = (const struct rfid_driver_api *)dev->api;

	return api->is_present(dev, atqa);
}

/**
 * @brief Select a card and get its UID.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param[out] uid Pointer to a buffer to store the UID.
 * @param[in,out] uid_len As input, the size of the UID buffer. As output, the actual UID length.
 * @param[out] sak Pointer to a buffer to store the SAK (Select Acknowledge) value.
 * @return 0 on success, negative error code on failure.
 */
static inline int rfid_select(const struct device *dev, uint8_t *uid, size_t *uid_len, uint8_t *sak)
{
	const struct rfid_driver_api *api = (const struct rfid_driver_api *)dev->api;

	return api->select(dev, uid, uid_len, sak);
}

/**
 * @brief Halt a card.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @return 0 on success, negative error code on failure.
 */
static inline int rfid_halt(const struct device *dev)
{
	const struct rfid_driver_api *api = (const struct rfid_driver_api *)dev->api;

	return api->halt(dev);
}

/**
 * @brief Send a RATS (Request for Answer to Select) command.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param[out] ats Pointer to a buffer to store the ATS (Answer to Select) response.
 * @param[in,out] ats_len As input, the size of the ATS buffer. As output, the actual ATS length.
 * @return 0 on success, negative error code on failure.
 */
static inline int rfid_rats(const struct device *dev, uint8_t *ats, size_t *ats_len)
{
	const struct rfid_driver_api *api = (const struct rfid_driver_api *)dev->api;

	return api->rats(dev, ats, ats_len);
}

/**
 * @brief Send a PPS (Protocol and Parameter Selection) command.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param dsi DSI (Divisor Send Integer) value for the PCD to PICC bitrate.
 * @param dri DRI (Divisor Receive Integer) value for the PICC to PCD bitrate.
 * @return 0 on success, negative error code on failure.
 */
static inline int rfid_pps(const struct device *dev, uint8_t dsi, uint8_t dri)
{
	const struct rfid_driver_api *api = (const struct rfid_driver_api *)dev->api;

	return api->pps(dev, dsi, dri);
}

/**
 * @brief Exchange data with the card.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param[in] tx_buf Pointer to the buffer containing data to transmit.
 * @param tx_len Length of the data to transmit.
 * @param[out] rx_buf Pointer to the buffer to store received data.
 * @param[in,out] rx_len As input, the size of the receive buffer. As output, the actual length of received data.
 * @return 0 on success, negative error code on failure.
 */
static inline int rfid_transceive(const struct device *dev, const uint8_t *tx_buf, size_t tx_len,
				  uint8_t *rx_buf, size_t *rx_len)
{
	const struct rfid_driver_api *api = (const struct rfid_driver_api *)dev->api;

	return api->transceive(dev, tx_buf, tx_len, rx_buf, rx_len);
}

/**
 * @brief Send a WTX (Waiting Time eXtension) command.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param wtxm WTX multiplier.
 * @return 0 on success, negative error code on failure.
 */
static inline int rfid_wtx(const struct device *dev, uint8_t wtxm)
{
	const struct rfid_driver_api *api = (const struct rfid_driver_api *)dev->api;

	return api->wtx(dev, wtxm);
}

/**
 * @brief Deselect a card.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @return 0 on success, negative error code on failure.
 */
static inline int rfid_deselect(const struct device *dev)
{
	const struct rfid_driver_api *api = (const struct rfid_driver_api *)dev->api;

	return api->deselect(dev);
}

/**
 * @brief Set a trigger.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param trig The trigger to set.
 * @param handler The function to call when the trigger fires.
 * @return 0 on success, negative error code on failure.
 */
static inline int rfid_trigger_set(const struct device *dev, enum rfid_trigger trig,
				   rfid_trigger_handler_t handler)
{
	const struct rfid_driver_api *api = (const struct rfid_driver_api *)dev->api;

	return api->trigger_set(dev, trig, handler);
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_RFID_H_ */