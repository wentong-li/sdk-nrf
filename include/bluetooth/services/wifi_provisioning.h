/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
/**
 * @file wifi_provisioning.h
 *
 * @brief WiFi Provisioning Service
 */
#ifndef WIFI_PROVISIONING_H_
#define WIFI_PROVISIONING_H_

#include <zephyr/bluetooth/uuid.h>
#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>

#ifdef __cplusplus
extern "C" {
#endif

/* WiFi Provisioning Service UUID */
#define BT_UUID_PROV_VAL \
	BT_UUID_128_ENCODE(0x14387800, 0x130c, 0x49e7, 0xb877, 0x2881c89cb258)
#define BT_UUID_PROV \
	BT_UUID_DECLARE_128(BT_UUID_PROV_VAL)

/* Information characteristic UUID */
#define BT_UUID_PROV_INFO_VAL \
	BT_UUID_128_ENCODE(0x14387801, 0x130c, 0x49e7, 0xb877, 0x2881c89cb258)
#define BT_UUID_PROV_INFO \
	BT_UUID_DECLARE_128(BT_UUID_PROV_INFO_VAL)

/* Control Point characteristic UUID */
#define BT_UUID_PROV_CONTROL_POINT_VAL \
	BT_UUID_128_ENCODE(0x14387802, 0x130c, 0x49e7, 0xb877, 0x2881c89cb258)
#define BT_UUID_PROV_CONTROL_POINT \
	BT_UUID_DECLARE_128(BT_UUID_PROV_CONTROL_POINT_VAL)

/* Data out characteristic UUID */
#define BT_UUID_PROV_DATA_OUT_VAL \
	BT_UUID_128_ENCODE(0x14387803, 0x130c, 0x49e7, 0xb877, 0x2881c89cb258)
#define BT_UUID_PROV_DATA_OUT \
	BT_UUID_DECLARE_128(BT_UUID_PROV_DATA_OUT_VAL)

/**
 * @def PROV_SVC_VER
 *
 * Firmware version.
 */
#define PROV_SVC_VER	0x01

/** Data structure to hold Wi-Fi credential */
struct wifi_config {
	/** String of SSID. */
	uint8_t                  ssid[WIFI_SSID_MAX_LEN];
	/** Length of SSID. */
	uint16_t                 ssid_len;
	/** Bytes of BSSID. */
	uint8_t                  bssid[WIFI_MAC_ADDR_LEN];
	/** String of password. */
	uint8_t                  password[WIFI_PSK_MAX_LEN];
	/** Length of password. */
	uint16_t                 password_len;
	/** Authendication method. */
	enum wifi_security_type  auth_type;
	/** Band of access point. */
	uint8_t                  band;
	/** Channel of access point. */
	uint8_t                  channel;
};

/**
 * @brief Check if there exists valid configuration (i.e., the device is provisioned).
 *
 * @return true if valid configuration found, false otherwise.
 */
bool wifi_has_config(void);

/**
 * @brief Get saved configuration.
 *
 * The value is copied to the pointer.
 * User must call wifi_has_config() first to test if a valid configuration exists.
 * The config must not be used if wifi_has_config() returns false.
 *
 * @return 0 if configuration retrieved successfully, negative error code otherwise.
 */
int wifi_get_config(struct wifi_config *config);

/**
 * @brief Save the configuration.
 *
 * If ram_only is set, the configuration will be saved in RAM only.
 * Otherwise, it will be saved in both RAM and Flash.
 *
 * @return 0 if configuration saved successfully, negative error code otherwise.
 */
int wifi_set_config(struct wifi_config *config, bool ram_only);

/**
 * @brief Mark the configuration as valid.
 *
 * Only valid configuration will lead to device state of "provisioned".
 *
 * @return 0 if configuration marked successfully, negative error code otherwise.
 */
int wifi_commit_config(void);

/**
 * @brief Remove the configuration.
 *
 * @return 0 if configuration removed successfully, negative error code otherwise.
 */
int wifi_remove_config(void);

/**
 * @brief Initialize the configuration module.
 *
 * @return 0 if module initialized successfully, negative error code otherwise.
 */
int wifi_config_init(void);

/**
 * @brief Initialize the provisioning module.
 *
 * @return 0 if module initialized successfully, negative error code otherwise.
 */
int wifi_prov_init(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_PROVISIONING_H_ */
