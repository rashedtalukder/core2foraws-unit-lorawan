/*!
 * @brief Library for the LoRaWAN915 (ASR6501) unit by
 * M5Stack on the Core2 for AWS
 *
 * @copyright Copyright (c) 2025 by Rashed Talukder[https://rashedtalukder.com]
 *
 * @license SPDX-License-Identifier: Apache 2.0
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @Links [Unit LoRaWAN915](https://docs.m5stack.com/en/unit/lorawan915)
 *
 * @version  V1.0.0
 * @date  2025-06-09
 */

#ifndef UNIT_LORAWAN_H
#define UNIT_LORAWAN_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// TTN US915 Configuration Constants
#define UNIT_LORAWAN_TTN_US915_RX2_FREQUENCY                                   \
  923300000 ///< TTN standard RX2 frequency for US915 (Hz)
#define UNIT_LORAWAN_TTN_US915_RX2_DATA_RATE                                   \
  8 ///< TTN standard RX2 data rate for US915
#define UNIT_LORAWAN_TTN_US915_SUB_BAND_DEFAULT                                \
  2 ///< TTN commonly uses sub-band 2 (channels 8-15)
#define UNIT_LORAWAN_TTN_US915_DATA_RATE_DEFAULT                               \
  2 ///< Recommended initial data rate for TTN US915
#define UNIT_LORAWAN_TTN_US915_ADR_ENABLED                                     \
  true ///< Enable ADR by default for TTN
#define UNIT_LORAWAN_TTN_US915_JOIN_TIMEOUT_SEC                                \
  60 ///< Default join timeout in seconds
#define UNIT_LORAWAN_TTN_V3_APP_EUI                                            \
  "0000000000000000" ///< Standard AppEUI for TTN v3

// LoRaWAN US915 Data Rate Constants
#define UNIT_LORAWAN_US915_DATA_RATE_MIN                                       \
  0 ///< Minimum US915 data rate (SF10, longest range)
#define UNIT_LORAWAN_US915_DATA_RATE_MAX                                       \
  4 ///< Maximum US915 data rate (SF8 500kHz, shortest range)

// LoRaWAN US915 Payload Size Constants (bytes)
#define UNIT_LORAWAN_US915_MAX_PAYLOAD_DR0                                     \
  11 ///< Maximum payload for DR0 (SF10, 125kHz)
#define UNIT_LORAWAN_US915_MAX_PAYLOAD_DR1                                     \
  53 ///< Maximum payload for DR1 (SF9, 125kHz)
#define UNIT_LORAWAN_US915_MAX_PAYLOAD_DR2                                     \
  125 ///< Maximum payload for DR2 (SF8, 125kHz)
#define UNIT_LORAWAN_US915_MAX_PAYLOAD_DR3                                     \
  242 ///< Maximum payload for DR3 (SF7, 125kHz)
#define UNIT_LORAWAN_US915_MAX_PAYLOAD_DR4                                     \
  242 ///< Maximum payload for DR4 (SF8, 500kHz)

// US915 Sub-band Constants
#define UNIT_LORAWAN_US915_SUB_BAND_MIN 1 ///< Minimum valid US915 sub-band
#define UNIT_LORAWAN_US915_SUB_BAND_MAX 8 ///< Maximum valid US915 sub-band

// General LoRaWAN Constants
#define UNIT_LORAWAN_LOG_LEVEL_MIN 0 ///< Minimum log level (no logging)
#define UNIT_LORAWAN_LOG_LEVEL_MAX 5 ///< Maximum log level (verbose)
#define UNIT_LORAWAN_EUI_LENGTH                                                \
  16 ///< Length of DevEUI and AppEUI in hex characters
#define UNIT_LORAWAN_APP_KEY_LENGTH 32 ///< Length of AppKey in hex characters

/**
 * @brief The maximum message size for sending LoRaWAN messages safely across
 * all data rates.
 *
 * This is set to the minimum payload size supported by TTN (DR0 = 11 bytes) to
 * ensure compatibility across all data rates. For higher data rates, larger
 * payloads are supported:
 * - DR0: 11 bytes (SF10 BW125)
 * - DR1: 53 bytes (SF9 BW125)
 * - DR2: 125 bytes (SF8 BW125)
 * - DR3: 242 bytes (SF7 BW125)
 * - DR4: 242 bytes (SF8 BW500)
 */
#define UNIT_LORAWAN_MAX_MESSAGE_SIZE 11

  /**
   * @brief Upload/download frequency modes for LoRaWAN communication.
   *
   * These modes determine how the device handles uplink and downlink
   * frequencies:
   * - DIFFERENT_FREQ_MODE: Uses different frequencies for uplink and downlink
   * (recommended for TTN)
   * - SAME_FREQ_MODE: Uses the same frequency for both uplink and downlink
   */
  typedef enum
  {
    DIFFERENT_FREQ_MODE =
        0, /**< Different frequencies for uplink/downlink (TTN standard) */
    SAME_FREQ_MODE = 1, /**< Same frequency for uplink/downlink */
  } unit_lorwan_uldlmode;

  /**
   * @brief TTN LoRaWAN configuration structure for US915 band
   */
  typedef struct
  {
    const char *dev_eui;
    const char *app_eui;
    const char *app_key;
    uint8_t sub_band;
    uint8_t data_rate;
    bool adr_enabled;
    uint32_t rx2_frequency;
    uint8_t rx2_data_rate;
    uint16_t join_timeout_sec;
  } unit_lorawan_ttn_config_t;

  /**
   * @brief Callback function type for TTN join status
   * @param joined True if join was successful, false if failed
   * @param error_code Error code if join failed (0 if successful, 1 for
   * timeout)
   * @param user_data User data passed to the callback
   */
  typedef void ( *unit_lorawan_ttn_join_callback_t )( bool joined,
                                                      uint8_t error_code,
                                                      void *user_data );

  /**
   * @brief Sets the log level for the LoRaWAN module.
   *
   * Controls the verbosity of debug output from the LoRaWAN module itself.
   * Lower levels reduce output and improve performance.
   *
   * @param level The log level to set:
   *              - 0: No logging
   *              - 1: Error messages only
   *              - 2: Error + info messages
   *              - 3: Error + info + debug messages
   *              - 4: Error + info + debug + verbose messages
   *              - 5: All messages
   *
   * @return
   * [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
   *  - ESP_OK                : Log level set successfully
   *  - ESP_FAIL              : Failed to communicate with module
   *  - ESP_ERR_INVALID_ARG   : Invalid log level (will be clamped to valid
   * range)
   */
  esp_err_t unit_lorawan_log( uint8_t level );

  /**
   * @brief Checks if the LoRaWAN device is connected to a network.
   *
   * Queries the module's connection status and provides human-readable status
   * information. A device must be connected to send/receive LoRaWAN messages.
   *
   * @param[out] state Pointer to boolean that will be set to:
   *                   - true: Device is joined and connected to network
   *                   - false: Device is not connected to network
   *
   * @return
   * [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
   *  - ESP_OK                : Successfully checked connection status
   *  - ESP_FAIL              : Failed to communicate with module
   *  - ESP_ERR_INVALID_ARG   : state parameter is NULL
   */
  esp_err_t unit_lorawan_connected( bool *state );

  /**
   * @brief Attempts to join the LoRaWAN network.
   *
   * Initiates the network join procedure using previously configured OTAA
   * credentials. The join process is asynchronous and may take up to 30 seconds
   * to complete. Use unit_lorawan_connected() to check if the join was
   * successful.
   *
   * @note The device must be configured with valid OTAA credentials before
   * calling this function.
   * @note Join attempts are subject to duty cycle limitations and may be
   * delayed.
   *
   * @return
   * [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
   *  - ESP_OK                : Join command sent successfully (does not
   * guarantee successful join)
   *  - ESP_FAIL              : Failed to send join command
   */
  esp_err_t unit_lorawan_join( void );

  /**
   * @brief Configures the LoRaWAN device for OTAA (Over-The-Air Activation).
   *
   * Sets up all necessary parameters for OTAA join procedure and configures
   * the device for compatibility with The Things Network (TTN) US915 band.
   *
   * @param devEUI Device EUI (16 hex characters, e.g., "0123456789ABCDEF")
   * @param appEUI Application EUI (16 hex characters, e.g., "FEDCBA9876543210")
   * @param appKey Application Key (32 hex characters)
   * @param mode Upload/download frequency mode (DIFFERENT_FREQ_MODE recommended
   * for TTN)
   *
   * @note All EUI and key strings must be valid hexadecimal without spaces or
   * delimiters.
   * @note This function automatically configures TTN US915 sub-band 2 (channels
   * 8-15).
   * @note The device will be set to LoRaWAN Class A with ADR enabled.
   *
   * @return
   * [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
   *  - ESP_OK                : OTAA configuration completed successfully
   *  - ESP_FAIL              : Failed to configure one or more parameters
   *  - ESP_ERR_INVALID_ARG   : NULL or invalid EUI/key parameters
   */
  esp_err_t unit_lorawan_configOTTA( char *devEUI, char *appEUI, char *appKey,
                                     unit_lorwan_uldlmode mode );

  /**
   * @brief Reboots the LoRaWAN module.
   *
   * Performs a soft reset of the LoRaWAN module. This is useful for:
   * - Applying configuration changes
   * - Recovering from error states
   * - Clearing internal buffers
   *
   * @note The function waits 2 seconds after sending the reboot command to
   * allow module restart.
   * @note All network connections will be lost and must be re-established after
   * reboot.
   *
   * @return
   * [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
   *  - ESP_OK                : Reboot command sent successfully
   *  - ESP_FAIL              : Failed to send reboot command
   */
  esp_err_t unit_lorawan_reboot( void );

  /**
   * @brief Checks if the LoRaWAN module is physically connected and responding.
   *
   * Verifies that the LoRaWAN module hardware is properly connected to the
   * expansion port and responding to UART commands. This is typically the
   * first check performed during initialization.
   *
   * @param[out] state Pointer to boolean that will be set to:
   *                   - true: Module is detected and responding
   *                   - false: Module is not detected or not responding
   *
   * @return
   * [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
   *  - ESP_OK                : Successfully checked module presence
   *  - ESP_FAIL              : Failed to communicate with module
   *  - ESP_ERR_INVALID_ARG   : state parameter is NULL
   */
  esp_err_t unit_lorawan_attached( bool *state );

  /**
   * @brief Sends an uplink message to the LoRaWAN network.
   *
   * Transmits a message to the LoRaWAN network as a confirmed uplink on port 1.
   * The message will be automatically converted to hexadecimal format for
   * transmission.
   *
   * @note Message size is validated against TTN payload limits for different
   * data rates.
   * @note The device must be connected to a network before sending messages.
   * @note Actual transmission timing depends on duty cycle limitations and
   * network conditions.
   *
   * @param message Pointer to the message data (ASCII format)
   * @param length Length of the message in bytes (max depends on data rate, see
   * UNIT_LORAWAN_MAX_MESSAGE_SIZE)
   *
   * @return
   * [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
   * - ESP_OK                : Message queued for transmission successfully
   * - ESP_FAIL              : Failed to send message
   * - ESP_ERR_INVALID_SIZE  : Message length exceeds TTN payload limits
   * - ESP_ERR_INVALID_ARG   : message parameter is NULL or length is 0
   * - ESP_ERR_NO_MEM        : Failed to allocate memory for message processing
   */
  esp_err_t unit_lorawan_send( char *message, size_t length );

  /**
   * @brief Initializes the LoRaWAN module driver.
   *
   * Performs complete initialization of the LoRaWAN module driver including:
   * - UART communication setup at 115200 baud
   * - Module presence detection and verification
   * - Initial configuration and module restart
   * - Buffer clearing and setup
   *
   * This function must be called before any other LoRaWAN operations.
   *
   * @note Requires the Core2 for AWS expansion port C to be available.
   * @note The function will wait up to 2 seconds for module initialization.
   * @note Detailed error messages will guide troubleshooting if initialization
   * fails.
   *
   * @return
   * [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
   *  - ESP_OK                : Driver initialized successfully
   *  - ESP_FAIL              : Module not detected or initialization failed
   */
  esp_err_t unit_lorawan_init( void );

  /**
   * @brief Initialize and configure LoRaWAN module for The Things Network (TTN)
   * US915
   *
   * This function provides a complete TTN-specific configuration for the US915
   * frequency band with all recommended settings for optimal performance and
   * compliance with TTN requirements.
   *
   * Key features:
   * - Automatic US915 sub-band configuration (typically sub-band 2 for TTN)
   * - TTN-specific RX2 frequency and data rate settings
   * - Adaptive Data Rate (ADR) enablement for optimal power consumption
   * - Proper channel mask configuration for TTN gateway compatibility
   * - Asynchronous join process with callback notification
   * - Comprehensive error handling and validation
   *
   * TTN US915 Configuration Details:
   * - Frequency Band: 902-928 MHz (US915)
   * - Sub-band: Configurable (1-8), TTN typically uses sub-band 2 (channels
   * 8-15)
   * - RX2 Frequency: 923.3 MHz (TTN standard for US915)
   * - RX2 Data Rate: DR8 (SF12, 500kHz)
   * - Default Data Rate: DR2 (SF10, 125kHz, ~125 bytes payload)
   * - Class: A (battery-optimized)
   *
   * @param[in] config TTN configuration structure with device credentials and
   * settings
   * @param[in] join_callback Optional callback function for asynchronous join
   * status (can be NULL)
   * @param[in] user_data Optional user data pointer passed to join callback
   * (can be NULL)
   *
   * @return
   *     - ESP_OK: TTN configuration completed successfully, join process
   * initiated
   *     - ESP_ERR_INVALID_ARG: Invalid configuration parameters
   *     - ESP_ERR_INVALID_STATE: LoRaWAN module not initialized or not
   * responding
   *     - ESP_FAIL: TTN configuration failed
   *
   * @note Device credentials (DevEUI, AppEUI, AppKey) must be obtained from TTN
   * Console
   * @note For TTN v3 stack, AppEUI should typically be all zeros:
   * "0000000000000000"
   * @note Join process is asynchronous - use callback or
   * unit_lorawan_connected() to check status
   * @note Ensure proper antenna for 915MHz band is connected before calling
   *
   * @warning This function will reboot the LoRaWAN module during configuration
   */
  esp_err_t unit_lorawan_configure_ttn_us915(
      const unit_lorawan_ttn_config_t *config,
      unit_lorawan_ttn_join_callback_t join_callback, void *user_data );

  /**
   * @brief Get current LoRaWAN data rate and maximum payload size
   *
   * Queries the current data rate setting and returns the maximum payload size
   * allowed for that data rate according to LoRaWAN regional parameters.
   *
   * @param[out] current_data_rate Current data rate (0-15, depending on region)
   * @param[out] max_payload_size Maximum payload size in bytes for current data
   * rate
   *
   * @return
   *     - ESP_OK: Data rate and payload size retrieved successfully
   *     - ESP_ERR_INVALID_ARG: NULL pointer parameters
   *     - ESP_ERR_INVALID_STATE: Module not responding or not configured
   *     - ESP_FAIL: Failed to query data rate
   */
  esp_err_t unit_lorawan_get_data_rate_info( uint8_t *current_data_rate,
                                             size_t *max_payload_size );

  /**
   * @brief Set LoRaWAN data rate
   *
   * Sets the transmission data rate for LoRaWAN communications. Higher data
   * rates allow larger payloads and faster transmission but have shorter range.
   *
   * US915 Data Rates:
   * - DR0: SF10, 125kHz, ~11 bytes max payload, longest range
   * - DR1: SF9, 125kHz, ~53 bytes max payload
   * - DR2: SF8, 125kHz, ~125 bytes max payload (recommended for TTN)
   * - DR3: SF7, 125kHz, ~242 bytes max payload
   * - DR4: SF8, 500kHz, ~242 bytes max payload, shortest range
   *
   * @param[in] data_rate Data rate to set (0-4 for US915)
   *
   * @return
   *     - ESP_OK: Data rate set successfully
   *     - ESP_ERR_INVALID_ARG: Invalid data rate for current region
   *     - ESP_ERR_INVALID_STATE: Module not responding or not joined to network
   *     - ESP_FAIL: Failed to set data rate
   *
   * @note When ADR is enabled, the network may override this setting
   * @note Higher data rates consume more power but transmit faster
   */
  esp_err_t unit_lorawan_set_data_rate( uint8_t data_rate );

  /**
   * @brief Set RX2 frequency for downlink reception
   *
   * Sets the frequency used for the second receive window (RX2) in Hz.
   * This is critical for TTN compliance in different regions.
   *
   * @param frequency RX2 frequency in Hz (e.g., 923300000 for TTN US915)
   *
   * @return
   *     - ESP_OK: RX2 frequency set successfully
   *     - ESP_ERR_INVALID_ARG: Invalid frequency value
   *     - ESP_FAIL: Failed to set RX2 frequency
   */
  esp_err_t unit_lorawan_set_rx2_frequency( uint32_t frequency );

  /**
   * @brief Set RX2 data rate for downlink reception
   *
   * Sets the data rate used for the second receive window (RX2).
   *
   * @param data_rate RX2 data rate (0-15, typically 8 for TTN US915)
   *
   * @return
   *     - ESP_OK: RX2 data rate set successfully
   *     - ESP_ERR_INVALID_ARG: Invalid data rate value
   *     - ESP_FAIL: Failed to set RX2 data rate
   */
  esp_err_t unit_lorawan_set_rx2_data_rate( uint8_t data_rate );

  /**
   * @brief Get current RSSI (Received Signal Strength Indicator)
   *
   * Retrieves the RSSI value of the last received packet.
   *
   * @param[out] rssi Pointer to store RSSI value in dBm
   *
   * @return
   *     - ESP_OK: RSSI retrieved successfully
   *     - ESP_ERR_INVALID_ARG: NULL pointer parameter
   *     - ESP_FAIL: Failed to get RSSI
   */
  esp_err_t unit_lorawan_get_rssi( int16_t *rssi );

  /**
   * @brief Get RSSI for all channels in a frequency band
   *
   * Scans and returns RSSI values for all 8 channels in the specified frequency
   * band. Per ASR6501 datasheet, this returns RSSI values for channels 0-7 in
   * the specified frequency group. Useful for network diagnostics and channel
   * selection.
   *
   * @param[in] freq_band_idx Frequency band index (0-based, where 1 = group
   * 1A2, etc.)
   * @param[out] rssi_values Array to store RSSI values (must be at least 8
   * elements for channels 0-7)
   * @param[out] channel_count Number of channels scanned (always 8 for valid
   * frequency bands)
   *
   * @return
   *     - ESP_OK: Channel RSSI values retrieved successfully
   *     - ESP_ERR_INVALID_ARG: Invalid parameters or frequency band index
   *     - ESP_FAIL: Failed to scan channels
   *
   * @note This function uses the AT+CRSSI FREQBANDIDX? command format per
   * datasheet
   * @note Returns RSSI values for all 8 channels (0-7) in the specified
   * frequency group
   */
  esp_err_t unit_lorawan_get_channel_rssi( uint8_t freq_band_idx,
                                           int16_t *rssi_values,
                                           size_t *channel_count );

  /**
   * @brief Set number of transmission retries
   *
   * Sets the maximum number of retransmission attempts for confirmed messages.
   *
   * @param message_type 0 for unconfirmed, 1 for confirmed messages
   * @param retries Number of retries (1-15)
   *
   * @return
   *     - ESP_OK: Retry count set successfully
   *     - ESP_ERR_INVALID_ARG: Invalid parameters
   *     - ESP_FAIL: Failed to set retry count
   */
  esp_err_t unit_lorawan_set_retries( uint8_t message_type, uint8_t retries );

  /**
   * @brief Set transmission power
   *
   * Sets the transmission power level in dBm.
   *
   * @param power_dbm Power level (0-7 for index, actual dBm depends on region)
   *                  US915: 0=30dBm, 1=28dBm, 2=26dBm, ..., 10=10dBm
   *
   * @return
   *     - ESP_OK: TX power set successfully
   *     - ESP_ERR_INVALID_ARG: Invalid power level
   *     - ESP_FAIL: Failed to set TX power
   */
  esp_err_t unit_lorawan_set_tx_power( uint8_t power_dbm );

  /**
   * @brief Get transmission power
   *
   * Gets the current transmission power setting.
   *
   * @param[out] power_dbm Pointer to store power level
   *
   * @return
   *     - ESP_OK: TX power retrieved successfully
   *     - ESP_ERR_INVALID_ARG: NULL pointer parameter
   *     - ESP_FAIL: Failed to get TX power
   */
  esp_err_t unit_lorawan_get_tx_power( uint8_t *power_dbm );

  /**
   * @brief Enable or disable link check mechanism
   *
   * Link check verifies network connectivity by requesting acknowledgment from
   * the network server.
   *
   * @param mode 0=disable, 1=check once, 2=check after every uplink
   *
   * @return
   *     - ESP_OK: Link check configured successfully
   *     - ESP_ERR_INVALID_ARG: Invalid mode
   *     - ESP_FAIL: Failed to configure link check
   */
  esp_err_t unit_lorawan_link_check( uint8_t mode );

  /**
   * @brief Save current configuration to non-volatile memory
   *
   * Saves all current LoRaWAN settings to flash/EEPROM so they persist after
   * reboot.
   *
   * @return
   *     - ESP_OK: Configuration saved successfully
   *     - ESP_FAIL: Failed to save configuration
   */
  esp_err_t unit_lorawan_save_config( void );

  /**
   * @brief Restore factory default configuration
   *
   * Resets all LoRaWAN settings to factory defaults.
   *
   * @return
   *     - ESP_OK: Configuration restored successfully
   *     - ESP_FAIL: Failed to restore configuration
   */
  esp_err_t unit_lorawan_restore_defaults( void );

  /**
   * @brief Send raw AT command
   *
   * Sends a raw AT command to the LoRaWAN module for advanced usage.
   *
   * @param[in] command AT command without "AT+" prefix
   * @param[out] response Buffer to store response (can be NULL)
   * @param[in] response_size Size of response buffer
   * @param[in] timeout_ms Timeout in milliseconds
   *
   * @return
   *     - ESP_OK: Command executed successfully
   *     - ESP_ERR_INVALID_ARG: Invalid parameters
   *     - ESP_ERR_TIMEOUT: Command timeout
   *     - ESP_FAIL: Command failed
   */
  esp_err_t unit_lorawan_send_raw_command( const char *command, char *response,
                                           size_t response_size,
                                           uint32_t timeout_ms );

  /**
   * @brief Initialize LoRaWAN with Kconfig values
   *
   * Initializes the LoRaWAN module using configuration values from Kconfig.
   * This provides a convenient way to use compile-time configuration without
   * hardcoding credentials in the application code.
   *
   * @param[in] join_callback Optional callback function for join status (can be
   * NULL)
   * @param[in] user_data Optional user data pointer passed to join callback
   * (can be NULL)
   *
   * @return
   *     - ESP_OK: Initialization and configuration completed successfully
   *     - ESP_ERR_INVALID_ARG: Invalid Kconfig values
   *     - ESP_ERR_INVALID_STATE: LoRaWAN module not responding
   *     - ESP_FAIL: Configuration or join failed
   *
   * @note This function uses CONFIG_LORAWAN_* values from Kconfig
   * @note Automatically detects OTAA vs ABP based on Kconfig selection
   * @note For US915, uses CONFIG_LORAWAN_US915_SUB_BAND and
   * CONFIG_LORAWAN_US915_DATA_RATE
   */
  esp_err_t
  unit_lorawan_init_with_config( unit_lorawan_ttn_join_callback_t join_callback,
                                 void *user_data );

  /**
   * @brief Configure LoRaWAN using Kconfig OTAA settings
   *
   * Configures the device for OTAA using compile-time Kconfig values.
   * Uses CONFIG_LORAWAN_DEVICE_EUI, CONFIG_LORAWAN_APP_EUI, and
   * CONFIG_LORAWAN_APP_KEY.
   *
   * @return
   *     - ESP_OK: OTAA configuration completed successfully
   *     - ESP_ERR_INVALID_ARG: Invalid or missing Kconfig values
   *     - ESP_FAIL: Configuration failed
   *
   * @note Requires CONFIG_LORAWAN_OTAA to be enabled in Kconfig
   */
  esp_err_t unit_lorawan_config_otaa_from_kconfig( void );

  /**
   * @brief Configure LoRaWAN using Kconfig ABP settings
   *
   * Configures the device for ABP using compile-time Kconfig values.
   * Uses CONFIG_LORAWAN_DEV_ADDR, CONFIG_LORAWAN_APP_SKEY, and
   * CONFIG_LORAWAN_NWK_SKEY.
   *
   * @return
   *     - ESP_OK: ABP configuration completed successfully
   *     - ESP_ERR_INVALID_ARG: Invalid or missing Kconfig values
   *     - ESP_ERR_NOT_SUPPORTED: ABP not fully implemented yet
   *     - ESP_FAIL: Configuration failed
   *
   * @note Requires CONFIG_LORAWAN_ABP to be enabled in Kconfig
   * @warning ABP support is experimental
   */
  esp_err_t unit_lorawan_config_abp_from_kconfig( void );

#ifdef __cplusplus
}
#endif

#endif // UNIT_LORAWAN_H