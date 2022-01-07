/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BLE_TRANS_H_
#define _BLE_TRANS_H_

#include <zephyr.h>
#include <bluetooth/conn.h>

/* Connection interval is calculated as x*1.25 */
#if (CONFIG_AUDIO_FRAME_DURATION_7_5_MS && CONFIG_SW_CODEC_LC3)
/* Connection interval of 7.5 ms */
#define BLE_ISO_CONN_INTERVAL 6
#else
/* Connection interval of 10 ms */
#define BLE_ISO_CONN_INTERVAL 8
#endif /* (CONFIG_AUDIO_FRAME_DURATION_7_5_MS && CONFIG_SW_CODEC_LC3) */

enum ble_trans_chan_type {
	BLE_TRANS_CHANNEL_RETURN_MONO = 0,
	BLE_TRANS_CHANNEL_LEFT = 0,
	BLE_TRANS_CHANNEL_RIGHT,
	BLE_TRANS_CHANNEL_STEREO,
	BLE_TRANS_CHANNEL_NUM,
};

enum iso_transport {
	TRANS_TYPE_NOT_SET, //< Default transport type. Not in use.
	TRANS_TYPE_BIS, //< Broadcast isochronous stream.
	TRANS_TYPE_CIS, //< Connected isochronous stream.
	TRANS_TYPE_NUM, //< Number of transport types.
};

enum iso_direction {
	DIR_NOT_SET,
	DIR_RX,
	DIR_TX,
	DIR_BIDIR,
	DIR_NUM,
};

/**@brief  BLE events
 */
enum ble_evt_type {
	BLE_EVT_CONNECTED,
	BLE_EVT_DISCONNECTED,
	BLE_EVT_LINK_READY,
	BLE_EVT_STREAMING,
	BLE_EVT_NUM_EVTS
};

/**@brief	BLE data callback type.
 *
 * @param data			Pointer to received data
 * @param size			Size of received data
 * @param bad_frame		Indicating if the frame is a bad frame or not
 * @param ts			ISO timestamp
 */
typedef void (*ble_trans_iso_rx_cb_t)(const uint8_t *const data, size_t size, bool bad_frame,
				      uint32_t ts);

/**@brief	Enable the ISO packet lost notify feature
 *
 * @return	0 for success, error otherwise.
 */
int ble_trans_iso_lost_notify_enable(void);

/**@brief	Send data over the ISO transport
 *		Could be either CIS or BIS dependent on configuration
 * @param data	Data to send
 * @param size	Size of data to send
 * @param chan_type Channel type (stereo or mono)
 *
 * @return	0 for success, error otherwise.
 */
int ble_trans_iso_tx(uint8_t const *const data, size_t size, enum ble_trans_chan_type chan_type);

/**@brief	Start iso stream
 *
 * @note	Type and direction is set by init
 *
 * @return	0 for success, error otherwise
 */
int ble_trans_iso_start(void);

/**@brief	Stop iso stream
 *
 * @note	Type and direction is set by init
 *
 * @return	0 for success, error otherwise
 */
int ble_trans_iso_stop(void);

/**
 * @brief    Trigger the scan for BIS
 *
 * @return	0 for success, error otherwise
 */
int ble_trans_iso_bis_rx_sync_get(void);

/**@brief Create ISO CIG
 *
 * @return 0 if successful, error otherwise
 */
int ble_trans_iso_cig_create(void);

/**@brief	Connect CIS ISO channel
 *
 * @param	conn ACL connection for CIS to connect
 *
 * @return	0 if successful, error otherwise
 */
int ble_trans_iso_cis_connect(struct bt_conn *conn);

/**@brief	Initialize either a CIS or BIS transport
 *
 * @return	0 for success, error otherwise
 */

int ble_trans_iso_init(enum iso_transport trans_type, enum iso_direction dir,
		       ble_trans_iso_rx_cb_t rx_cb);

#endif /* _BLE_TRANS_H_ */
