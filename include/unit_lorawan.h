/*!
 * @file unit_lorawan.h
 * @brief Driver for the M5Stack Unit LoRaWAN915 (ASR6501, US915) on Port C of
 * the Core2 for AWS IoT Kit.
 *
 * @copyright Copyright 2024-2026 Rashed Talukder (https://rashedtalukder.com)
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * The modem is driven over the BSP's Port C UART (GPIO 14 TX, GPIO 13 RX) with
 * the ASR650X AT command set. A driver-owned service task parses unsolicited
 * modem output, so join results, downlinks, and link-check answers are
 * delivered through one event callback even when no command is running.
 *
 * All functions are thread safe. Commands are serialized; blocking calls hold
 * the modem only for the duration of their own AT transaction. The event
 * callback runs on the service task and may call any driver function.
 *
 * @code{c}
 *   static void on_event( const unit_lorawan_event_t *event, void *ctx )
 *   {
 *       if( event->id == UNIT_LORAWAN_EVENT_DOWNLINK )
 *           handle( event->downlink.port, event->downlink.data,
 *                   event->downlink.length );
 *   }
 *
 *   unit_lorawan_config_t config = UNIT_LORAWAN_CONFIG_DEFAULT();
 *   config.event_cb = on_event;
 *   ESP_ERROR_CHECK( unit_lorawan_init( &config ) );
 *
 *   unit_lorawan_network_config_t network;
 *   ESP_ERROR_CHECK( unit_lorawan_network_config_from_kconfig( &network ) );
 *   ESP_ERROR_CHECK( unit_lorawan_configure( &network ) );
 *   ESP_ERROR_CHECK( unit_lorawan_join( NULL ) );
 *   ESP_ERROR_CHECK( unit_lorawan_wait_joined( 120000 ) );
 *
 *   const uint8_t payload[] = { 0x01, 0x02 };
 *   unit_lorawan_tx_result_t result;
 *   unit_lorawan_send( payload, sizeof( payload ), NULL, &result );
 * @endcode
 *
 * @see [Unit LoRaWAN915](https://docs.m5stack.com/en/unit/lorawan915)
 * @version 1.0.0
 * @date 2026-09-26
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

/** @brief Default modem UART rate (8N1). */
#define UNIT_LORAWAN_DEFAULT_BAUD 115200

/** @brief Hex characters in a DevEUI or AppEUI. */
#define UNIT_LORAWAN_EUI_HEX_LEN 16
/** @brief Hex characters in an AppKey, AppSKey, or NwkSKey. */
#define UNIT_LORAWAN_KEY_HEX_LEN 32
/** @brief Hex characters in a DevAddr. */
#define UNIT_LORAWAN_DEVADDR_HEX_LEN 8

/** @brief Largest uplink application payload on US915 (DR3/DR4). */
#define UNIT_LORAWAN_MAX_UPLINK 242
/** @brief Largest downlink the modem can report (one-byte LEN field). */
#define UNIT_LORAWAN_MAX_DOWNLINK 255

/** @brief Lowest and highest US915 uplink data rates (DR0 SF10 .. DR4 SF8/500). */
#define UNIT_LORAWAN_US915_DR_MIN 0
#define UNIT_LORAWAN_US915_DR_MAX 4

/** @brief US915 frequency plan bounds, used to reject out-of-band settings. */
#define UNIT_LORAWAN_US915_FREQ_MIN_HZ 902000000UL
#define UNIT_LORAWAN_US915_FREQ_MAX_HZ 928000000UL

/** @brief Standard US915 RX2 window (923.3 MHz, DR8). */
#define UNIT_LORAWAN_US915_RX2_FREQ_HZ 923300000UL
#define UNIT_LORAWAN_US915_RX2_DR 8

/**
 * @brief Channel mask for one US915 sub-band.
 *
 * Sub-band n (1..8) covers 125 kHz uplink channels 8(n-1)..8(n-1)+7.
 * TTN and most North American networks use sub-band 2.
 */
#define UNIT_LORAWAN_US915_SUB_BAND( n ) ( (uint16_t)( 1U << ( ( n ) - 1U ) ) )

/** @brief Leave a numeric setting unchanged in configuration structures. */
#define UNIT_LORAWAN_KEEP 0xFF

  /** @brief LoRaWAN device class. */
  typedef enum
  {
    UNIT_LORAWAN_CLASS_A = 0,
    UNIT_LORAWAN_CLASS_B = 1,
    UNIT_LORAWAN_CLASS_C = 2,
  } unit_lorawan_class_t;

  /** @brief Network activation method. */
  typedef enum
  {
    UNIT_LORAWAN_OTAA = 0,
    UNIT_LORAWAN_ABP = 1,
  } unit_lorawan_activation_t;

  /** @brief Driver-tracked join state. */
  typedef enum
  {
    UNIT_LORAWAN_JOIN_IDLE = 0,
    UNIT_LORAWAN_JOIN_IN_PROGRESS,
    UNIT_LORAWAN_JOIN_JOINED,
    UNIT_LORAWAN_JOIN_FAILED,
  } unit_lorawan_join_state_t;

  /** @brief Modem status reported by AT+CSTATUS. */
  typedef enum
  {
    UNIT_LORAWAN_STATUS_IDLE = 0,
    UNIT_LORAWAN_STATUS_SENDING = 1,
    UNIT_LORAWAN_STATUS_SEND_FAILED = 2,
    UNIT_LORAWAN_STATUS_SEND_OK = 3,
    UNIT_LORAWAN_STATUS_JOIN_OK = 4,
    UNIT_LORAWAN_STATUS_JOIN_FAILED = 5,
    UNIT_LORAWAN_STATUS_NETWORK_ABNORMAL = 6,
    UNIT_LORAWAN_STATUS_SEND_OK_NO_DOWNLINK = 7,
    UNIT_LORAWAN_STATUS_SEND_OK_DOWNLINK = 8,
  } unit_lorawan_status_t;

  /** @brief Link-check scheduling for unit_lorawan_link_check(). */
  typedef enum
  {
    UNIT_LORAWAN_LINK_CHECK_OFF = 0,
    UNIT_LORAWAN_LINK_CHECK_ONCE = 1,
    UNIT_LORAWAN_LINK_CHECK_EVERY_UPLINK = 2,
  } unit_lorawan_link_check_mode_t;

  /** @brief AT+IREBOOT modes. */
  typedef enum
  {
    UNIT_LORAWAN_REBOOT_NOW = 0,
    UNIT_LORAWAN_REBOOT_AFTER_TX = 1,
    UNIT_LORAWAN_REBOOT_BOOTLOADER = 7,
  } unit_lorawan_reboot_mode_t;

  /** @brief Outcome of an uplink, see unit_lorawan_send(). */
  typedef enum
  {
    UNIT_LORAWAN_TX_OK = 0,       /**< OK+SENT (and ACK for confirmed) */
    UNIT_LORAWAN_TX_NOT_JOINED,   /**< ERR+SEND:0 */
    UNIT_LORAWAN_TX_BUSY,         /**< ERR+SEND:1 */
    UNIT_LORAWAN_TX_TOO_LONG,     /**< ERR+SEND:2; only MAC commands were sent */
    UNIT_LORAWAN_TX_NO_ACK,       /**< ERR+SENT; trials exhausted */
    UNIT_LORAWAN_TX_REJECTED,     /**< ERROR or +CME ERROR */
    UNIT_LORAWAN_TX_TIMEOUT,      /**< No result from the modem in time */
  } unit_lorawan_tx_status_t;

/** @name Downlink TYPE flags (OK+RECV bit 0..3) */
/** @{ */
#define UNIT_LORAWAN_DL_CONFIRMED      0x01
#define UNIT_LORAWAN_DL_ACK            0x02
#define UNIT_LORAWAN_DL_LINK_CHECK_ACK 0x04
#define UNIT_LORAWAN_DL_TIME_ACK       0x08
/** @} */

  /** @brief Received downlink. Port 0 frames carry only MAC data. */
  typedef struct
  {
    uint8_t port;
    uint8_t flags; /**< UNIT_LORAWAN_DL_* bits */
    uint8_t length;
    uint8_t data[ UNIT_LORAWAN_MAX_DOWNLINK ];
  } unit_lorawan_downlink_t;

  /** @brief LinkCheckAns contents. */
  typedef struct
  {
    bool success;
    uint8_t demod_margin_db;
    uint8_t gateways;
    int16_t rssi_dbm;
    int8_t snr_db;
  } unit_lorawan_link_check_t;

  /** @brief Event identifiers. */
  typedef enum
  {
    UNIT_LORAWAN_EVENT_JOINED = 0,
    UNIT_LORAWAN_EVENT_JOIN_FAILED,
    UNIT_LORAWAN_EVENT_DOWNLINK,
    UNIT_LORAWAN_EVENT_LINK_CHECK,
  } unit_lorawan_event_id_t;

  /** @brief Event passed to the callback; valid only during the call. */
  typedef struct
  {
    unit_lorawan_event_id_t id;
    union
    {
      unit_lorawan_downlink_t downlink;     /**< EVENT_DOWNLINK */
      unit_lorawan_link_check_t link_check; /**< EVENT_LINK_CHECK */
      bool join_timed_out;                  /**< EVENT_JOIN_FAILED */
    };
  } unit_lorawan_event_t;

  typedef void ( *unit_lorawan_event_cb_t )( const unit_lorawan_event_t *event,
                                             void *ctx );

  /** @brief Driver settings for unit_lorawan_init(). */
  typedef struct
  {
    uint32_t baud;               /**< Modem UART rate; 0 = 115200 */
    unit_lorawan_event_cb_t event_cb; /**< May be NULL */
    void *event_ctx;
    uint32_t task_stack_size;    /**< Service task stack in bytes */
    uint8_t task_priority;
  } unit_lorawan_config_t;

#define UNIT_LORAWAN_CONFIG_DEFAULT()                                          \
  { .baud = UNIT_LORAWAN_DEFAULT_BAUD, .event_cb = NULL, .event_ctx = NULL,    \
    .task_stack_size = 4096, .task_priority = 5 }

  /**
   * @brief Network settings applied by unit_lorawan_configure().
   *
   * Credential strings are hexadecimal without delimiters. Fields set to
   * UNIT_LORAWAN_KEEP (or 0 where noted) leave the modem's value untouched.
   */
  typedef struct
  {
    unit_lorawan_activation_t activation;
    const char *dev_eui;  /**< OTAA, 16 hex */
    const char *app_eui;  /**< OTAA, 16 hex (JoinEUI; zeros for TTN v3) */
    const char *app_key;  /**< OTAA, 32 hex */
    const char *dev_addr; /**< ABP, 8 hex */
    const char *app_skey; /**< ABP, 32 hex */
    const char *nwk_skey; /**< ABP, 32 hex */
    uint16_t channel_mask; /**< 8-channel groups; 0 = keep */
    unit_lorawan_class_t device_class;
    bool adr;
    uint8_t data_rate;     /**< 0..4 or UNIT_LORAWAN_KEEP */
    uint8_t tx_power;      /**< Modem power index or UNIT_LORAWAN_KEEP */
    bool confirmed;        /**< Default uplink type */
    uint8_t confirmed_trials;   /**< 1..15 */
    uint8_t unconfirmed_trials; /**< 1..15 */
    uint8_t app_port;      /**< 1..223 */
    uint32_t rx2_frequency_hz; /**< 0 = keep the regional default */
    uint8_t rx2_data_rate;
    uint8_t rx1_dr_offset;
    bool save;             /**< Persist with AT+CSAVE (writes modem flash) */
  } unit_lorawan_network_config_t;

  /** @brief Join settings; see AT+CJOIN. */
  typedef struct
  {
    uint8_t period_s;      /**< Seconds between attempts, 7..255 */
    uint16_t max_attempts; /**< 1..256 */
    bool auto_join;
    uint32_t timeout_ms;   /**< Driver timeout; 0 = derive from attempts */
  } unit_lorawan_join_params_t;

#define UNIT_LORAWAN_JOIN_PARAMS_DEFAULT()                                     \
  { .period_s = 8, .max_attempts = 8, .auto_join = false, .timeout_ms = 0 }

  /** @brief Per-uplink options; NULL selects the configured defaults. */
  typedef struct
  {
    bool confirmed;
    uint8_t trials;      /**< 1..15; 0 = configured default */
    uint8_t port;        /**< 1..223; 0 = current port */
    uint32_t timeout_ms; /**< 0 = derive from trials */
  } unit_lorawan_tx_params_t;

  /** @brief Uplink outcome. */
  typedef struct
  {
    unit_lorawan_tx_status_t status;
    uint8_t transmissions; /**< TX_CNT from OK+SENT / ERR+SENT */
    bool acked;            /**< Downlink ACK bit seen during the uplink */
    bool downlink;         /**< A downlink arrived during the uplink */
  } unit_lorawan_tx_result_t;

  /** @brief Modem identity. */
  typedef struct
  {
    char manufacturer[ 16 ];
    char model[ 16 ];
    char revision[ 32 ];
    char serial[ 32 ];
  } unit_lorawan_info_t;

  /** @brief Class B parameters for unit_lorawan_set_class_b(). */
  typedef struct
  {
    bool custom;               /**< false: periodicity only (branch 0) */
    uint8_t periodicity;       /**< 0..7, ping every 0.96 * 2^n s */
    uint32_t beacon_frequency_hz; /**< custom only */
    uint8_t beacon_data_rate;     /**< custom only */
    uint32_t ping_frequency_hz;   /**< custom only */
    uint8_t ping_data_rate;       /**< custom only */
  } unit_lorawan_class_b_t;

  /** @brief Multicast group for unit_lorawan_multicast_add(). */
  typedef struct
  {
    const char *dev_addr; /**< 8 hex */
    const char *app_skey; /**< 32 hex */
    const char *nwk_skey; /**< 32 hex */
    bool class_b;         /**< Include periodicity and data rate */
    uint8_t periodicity;
    uint8_t data_rate;
  } unit_lorawan_multicast_t;

  /** @brief RX window parameters (AT+CRXP). */
  typedef struct
  {
    uint8_t rx1_dr_offset;
    uint8_t rx2_data_rate;
    uint32_t rx2_frequency_hz;
  } unit_lorawan_rx_params_t;

  /** @brief Driver counters for diagnostics. */
  typedef struct
  {
    uint32_t commands;
    uint32_t command_errors;
    uint32_t command_timeouts;
    uint32_t uplinks;
    uint32_t downlinks;
    uint32_t events_dropped;
    uint32_t lines_dropped; /**< Oversized or malformed lines */
  } unit_lorawan_stats_t;

  /* ---- Lifecycle ------------------------------------------------------- */

  /**
   * @brief Start Port C UART, probe the modem, and start the service task.
   *
   * Probes the configured baud first, then 9600..57600, so a modem left at
   * another rate is still found. Disables modem log output so it cannot
   * interleave with replies.
   *
   * @param[in] config Driver settings, or NULL for defaults.
   * @return ESP_OK, ESP_ERR_INVALID_STATE (already initialized),
   *         ESP_ERR_NOT_FOUND (no ASR650X answered), or a BSP/FreeRTOS error.
   */
  esp_err_t unit_lorawan_init( const unit_lorawan_config_t *config );

  /** @brief Stop the service task and release Port C. */
  esp_err_t unit_lorawan_deinit( void );

  /** @brief Replace the event callback. */
  esp_err_t unit_lorawan_set_event_callback( unit_lorawan_event_cb_t cb,
                                             void *ctx );

  /** @brief Read cached identity (CGMI/CGMM/CGMR/CGSN) from init. */
  esp_err_t unit_lorawan_get_info( unit_lorawan_info_t *info );

  /** @brief Current UART baud used to talk to the modem. */
  uint32_t unit_lorawan_get_baud( void );

  /**
   * @brief Change the modem UART rate (AT+CGBR) and follow it on Port C.
   * @param baud 9600, 19200, 38400, 57600, or 115200.
   */
  esp_err_t unit_lorawan_set_baud( uint32_t baud );

  /** @brief Copy the driver counters. */
  esp_err_t unit_lorawan_get_stats( unit_lorawan_stats_t *stats );

  /* ---- Configuration ---------------------------------------------------- */

  /**
   * @brief Fill a network configuration from Kconfig
   * (Component config -> Unit LoRaWAN).
   */
  esp_err_t
  unit_lorawan_network_config_from_kconfig( unit_lorawan_network_config_t *cfg );

  /**
   * @brief Apply a complete US915 network configuration.
   *
   * Stops any running join, then writes activation credentials, channel mask,
   * different-frequency UL/DL mode, normal work mode, class, ADR, data rate,
   * power, trials, port, and RX2 settings. Validates everything first so a
   * bad field never leaves the modem half configured.
   */
  esp_err_t
  unit_lorawan_configure( const unit_lorawan_network_config_t *cfg );

  /** @brief Set OTAA credentials and select OTAA. */
  esp_err_t unit_lorawan_set_otaa( const char *dev_eui, const char *app_eui,
                                   const char *app_key );

  /** @brief Set ABP session and select ABP. */
  esp_err_t unit_lorawan_set_abp( const char *dev_addr, const char *app_skey,
                                  const char *nwk_skey );

  /** @brief Read the modem DevEUI (factory value or last set). */
  esp_err_t unit_lorawan_get_dev_eui( char dev_eui[ UNIT_LORAWAN_EUI_HEX_LEN + 1 ] );

  /** @brief Read the AppEUI / JoinEUI. */
  esp_err_t unit_lorawan_get_app_eui( char app_eui[ UNIT_LORAWAN_EUI_HEX_LEN + 1 ] );

  /** @brief Read the DevAddr (assigned after OTAA join, or ABP value). */
  esp_err_t
  unit_lorawan_get_dev_addr( char dev_addr[ UNIT_LORAWAN_DEVADDR_HEX_LEN + 1 ] );

  /** @brief Read the selected activation method. */
  esp_err_t unit_lorawan_get_activation( unit_lorawan_activation_t *activation );

  /** @brief Enable 8-channel groups (AT+CFREQBANDMASK). Set before joining. */
  esp_err_t unit_lorawan_set_channel_mask( uint16_t mask );

  /** @brief Read the channel group mask. */
  esp_err_t unit_lorawan_get_channel_mask( uint16_t *mask );

  /** @brief Select Class A or C. Use unit_lorawan_set_class_b() for B. */
  esp_err_t unit_lorawan_set_class( unit_lorawan_class_t device_class );

  /** @brief Select Class B with ping-slot settings. */
  esp_err_t unit_lorawan_set_class_b( const unit_lorawan_class_b_t *params );

  /** @brief Read the device class. */
  esp_err_t unit_lorawan_get_class( unit_lorawan_class_t *device_class );

  /** @brief Send PingSlotInfoReq (Class B only). */
  esp_err_t unit_lorawan_ping_slot_info_request( uint8_t periodicity );

  /** @brief Enable or disable ADR. */
  esp_err_t unit_lorawan_set_adr( bool enabled );

  /** @brief Read the ADR setting. */
  esp_err_t unit_lorawan_get_adr( bool *enabled );

  /** @brief Set the uplink data rate (0..4). Ignored by the modem under ADR. */
  esp_err_t unit_lorawan_set_data_rate( uint8_t data_rate );

  /** @brief Read the uplink data rate. */
  esp_err_t unit_lorawan_get_data_rate( uint8_t *data_rate );

  /** @brief US915 maximum application payload at a data rate; 0 if invalid. */
  size_t unit_lorawan_max_payload( uint8_t data_rate );

  /**
   * @brief Set the modem TX power index (AT+CTXP, 0 = highest).
   *
   * The index-to-dBm table is defined by the modem firmware. US915 LoRaWAN
   * indices step 2 dB down from the 30 dBm EIRP ceiling; this module's
   * published conducted maximum is +21 dBm. Out-of-range indices are
   * rejected by the modem.
   */
  esp_err_t unit_lorawan_set_tx_power( uint8_t index );

  /** @brief Read the TX power index. */
  esp_err_t unit_lorawan_get_tx_power( uint8_t *index );

  /** @brief Set the default uplink type (AT+CCONFIRM). */
  esp_err_t unit_lorawan_set_confirmed( bool confirmed );

  /** @brief Read the default uplink type. */
  esp_err_t unit_lorawan_get_confirmed( bool *confirmed );

  /** @brief Set the uplink port (1..223). */
  esp_err_t unit_lorawan_set_port( uint8_t port );

  /** @brief Read the uplink port. */
  esp_err_t unit_lorawan_get_port( uint8_t *port );

  /** @brief Set transmissions per uplink (1..15) for one uplink type. */
  esp_err_t unit_lorawan_set_trials( bool confirmed, uint8_t trials );

  /** @brief Read transmissions per uplink for one uplink type. */
  esp_err_t unit_lorawan_get_trials( bool confirmed, uint8_t *trials );

  /** @brief Set RX1 offset and RX2 window (AT+CRXP). */
  esp_err_t unit_lorawan_set_rx_params( const unit_lorawan_rx_params_t *params );

  /** @brief Read RX window parameters. */
  esp_err_t unit_lorawan_get_rx_params( unit_lorawan_rx_params_t *params );

  /** @brief Set the RX1 delay in seconds (1..15). */
  esp_err_t unit_lorawan_set_rx1_delay( uint8_t seconds );

  /** @brief Read the RX1 delay in seconds. */
  esp_err_t unit_lorawan_get_rx1_delay( uint8_t *seconds );

  /**
   * @brief Enable modem periodic reporting (AT+CRM). Intended for testing.
   * @param periodic false disables; interval_s is ignored then.
   */
  esp_err_t unit_lorawan_set_report_mode( bool periodic, uint16_t interval_s );

  /** @brief Persist MAC configuration to modem flash (AT+CSAVE). */
  esp_err_t unit_lorawan_save( void );

  /** @brief Write modem MAC defaults to flash (AT+CRESTORE). Reboot after. */
  esp_err_t unit_lorawan_restore_defaults( void );

  /* ---- Join and data ---------------------------------------------------- */

  /**
   * @brief Start an OTAA join (AT+CJOIN). Returns once the modem accepts it.
   *
   * The result arrives as EVENT_JOINED or EVENT_JOIN_FAILED; the driver
   * raises JOIN_FAILED with join_timed_out set if the modem stays silent.
   *
   * @param[in] params Join settings, or NULL for defaults.
   */
  esp_err_t unit_lorawan_join( const unit_lorawan_join_params_t *params );

  /** @brief Stop a running join (AT+CJOIN=0). */
  esp_err_t unit_lorawan_join_stop( void );

  /**
   * @brief Block until joined.
   * @return ESP_OK, ESP_FAIL (join failed), ESP_ERR_TIMEOUT, or
   *         ESP_ERR_INVALID_STATE (no join started).
   */
  esp_err_t unit_lorawan_wait_joined( uint32_t timeout_ms );

  /** @brief Driver join state (JOINED also after ABP configuration). */
  unit_lorawan_join_state_t unit_lorawan_get_join_state( void );

  /** @brief Read AT+CSTATUS. */
  esp_err_t unit_lorawan_get_status( unit_lorawan_status_t *status );

  /**
   * @brief Send an uplink and wait for the modem's result.
   *
   * Blocks until OK+SENT, an error line, or the timeout. An empty payload
   * (length 0) flushes pending MAC commands.
   *
   * @param[in] data Payload bytes (may be NULL when length is 0).
   * @param[in] length Bytes, at most unit_lorawan_max_payload() for the DR.
   * @param[in] params Per-uplink options, or NULL for defaults.
   * @param[out] result Detailed outcome (may be NULL).
   * @return ESP_OK, ESP_ERR_INVALID_STATE (not joined), ESP_ERR_NOT_FINISHED
   *         (modem busy), ESP_ERR_INVALID_SIZE (too long for the DR),
   *         ESP_ERR_TIMEOUT (no ACK or no answer), or ESP_FAIL.
   */
  esp_err_t unit_lorawan_send( const uint8_t *data, size_t length,
                               const unit_lorawan_tx_params_t *params,
                               unit_lorawan_tx_result_t *result );

  /**
   * @brief Read and clear the modem RX buffer (AT+DRX?).
   * @param[out] length Bytes copied; 0 when the buffer is empty.
   */
  esp_err_t unit_lorawan_read_rx_buffer( uint8_t *data, size_t capacity,
                                         size_t *length );

  /* ---- Diagnostics ------------------------------------------------------ */

  /**
   * @brief Request a LinkCheck. Answers arrive as EVENT_LINK_CHECK.
   */
  esp_err_t unit_lorawan_link_check( unit_lorawan_link_check_mode_t mode );

  /** @brief Last LinkCheckAns; ESP_ERR_NOT_FOUND before the first answer. */
  esp_err_t unit_lorawan_get_last_link_check( unit_lorawan_link_check_t *out );

  /**
   * @brief Measure RSSI on the 8 channels of one group (AT+CRSSI).
   * @param group 0-based group index; US915 sub-band n is group n-1.
   */
  esp_err_t unit_lorawan_get_channel_rssi( uint8_t group, int16_t rssi_dbm[ 8 ] );

  /** @brief Read the battery level the modem reports in DevStatusAns. */
  esp_err_t unit_lorawan_get_battery_level( uint8_t *level );

  /** @brief Set modem log verbosity (0 off .. 5). Logs share the AT UART. */
  esp_err_t unit_lorawan_set_log_level( uint8_t level );

  /* ---- Multicast -------------------------------------------------------- */

  /** @brief Add a multicast group. Call before joining. */
  esp_err_t unit_lorawan_multicast_add( const unit_lorawan_multicast_t *group );

  /** @brief Remove a multicast group by DevAddr. */
  esp_err_t unit_lorawan_multicast_remove( const char *dev_addr );

  /** @brief Number of multicast groups. */
  esp_err_t unit_lorawan_multicast_count( uint8_t *count );

  /* ---- Power and reset -------------------------------------------------- */

  /**
   * @brief Enable or disable modem low-power mode (AT+CLPM).
   *
   * While enabled the driver prefixes each command with the documented wake
   * sequence (00 00 00 00 0D 0A).
   */
  esp_err_t unit_lorawan_set_low_power( bool enabled );

  /**
   * @brief Reboot the modem and wait until it answers again.
   *
   * Join state resets to IDLE. BOOTLOADER mode does not wait and leaves the
   * driver unusable until the modem is power-cycled and re-initialized.
   */
  esp_err_t unit_lorawan_reboot( unit_lorawan_reboot_mode_t mode );

  /**
   * @brief Permanently encrypt the stored credentials (AT+CKEYSPROTECT).
   *
   * @warning Irreversible. Credentials can never be changed afterwards.
   */
  esp_err_t unit_lorawan_protect_keys_irreversible( const char *key );

  /** @brief Whether key protection is active. */
  esp_err_t unit_lorawan_keys_protected( bool *protected_keys );

  /* ---- Factory tests ---------------------------------------------------- */
  /*
   * These commands put the modem into a loop that ignores AT commands. The
   * driver then refuses further commands; power-cycle the unit and call
   * unit_lorawan_deinit() and unit_lorawan_init(). TX tests are limited to
   * 902-928 MHz. Radiating a continuous carrier requires an appropriate
   * test environment.
   */

  /** @brief Continuous receive (AT+CRX). data_rate 0..5 = SF12..SF7. */
  esp_err_t unit_lorawan_test_rx( uint32_t frequency_hz, uint8_t data_rate );

  /** @brief Transmit a packet every second (AT+CTX). power_dbm 0..22. */
  esp_err_t unit_lorawan_test_tx( uint32_t frequency_hz, uint8_t data_rate,
                                  uint8_t power_dbm );

  /** @brief Continuous-wave carrier (AT+CTXCW). pa_option 0..3. */
  esp_err_t unit_lorawan_test_tx_cw( uint32_t frequency_hz, uint8_t power_dbm,
                                     uint8_t pa_option );

  /**
   * @brief Low-power test commands (AT+CSLEEP / AT+CMCU / AT+CSTDBY).
   *
   * Sleep mode 0 wakes after 10 s; the driver treats these as test modes.
   */
  esp_err_t unit_lorawan_test_sleep( uint8_t mode );
  esp_err_t unit_lorawan_test_mcu( uint8_t mode );
  esp_err_t unit_lorawan_test_standby( uint8_t mode );

  /* ---- Raw access ------------------------------------------------------- */

  /**
   * @brief Run one AT command and return its reply lines.
   *
   * @param[in] command Text after "AT+", e.g. "CGMR?". CR/LF are rejected.
   * @param[out] reply Reply lines joined with '\n' (may be NULL).
   * @param[in] timeout_ms Time allowed for the final OK/ERROR; 0 = default.
   * @return ESP_OK on OK, ESP_FAIL on ERROR, ESP_ERR_TIMEOUT,
   *         ESP_ERR_INVALID_SIZE if reply was truncated.
   */
  esp_err_t unit_lorawan_command( const char *command, char *reply,
                                  size_t reply_size, uint32_t timeout_ms );

#ifdef __cplusplus
}
#endif

#endif
