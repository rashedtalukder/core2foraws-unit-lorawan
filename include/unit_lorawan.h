/*!
 * @brief Library for the LoRaWAN915 unit by 
 * M5Stack on the Core2 for AWS
 * 
 * @copyright Copyright (c) 2024 by Rashed Talukder[https://rashedtalukder.com]
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
 * @Links [PAHUB](https://docs.m5stack.com/en/unit/pahub)
 * @Links [PAHUB2](https://docs.m5stack.com/en/unit/pahub2)
 * 
 * @version  V0.0.1
 * @date  2024-10-04
 */

#ifndef _UNIT_LORAWAN_H_
#define _UNIT_LORAWAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_err.h>

/** 
 * @brief The maximum message size for sending LoRaWAN messages.
 */
/* @[declare_unit_lorawan_max_message_size] */
#define UNIT_LORAWAN_MAX_MESSAGE_SIZE 8
/* @[declare_unit_lorawan_max_message_size] */

/** 
 * @brief Upload/download frequency modes.
 */
/* @[declare_unit_lorwan_uldlmode] */
typedef enum
{
    DIFFERENT_FREQ_MODE = 0,
    SAME_FREQ_MODE,
}unit_lorwan_uldlmode;
/* @[declare_unit_lorwan_uldlmode] */

/** 
 * @brief Sets the log level for the LoRaWAN unit.
 * 
 * @param level The log level to set. 0 = none, 1 = error, 2 = info, 3 = debug, 4=verbose, 5=all.
 * @return [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
 *  - ESP_OK                : Success
 *  - ESP_FAIL              : Failed to check LoRaWAN connection state.
 */
/* @[declare_unit_lorawan_log] */
esp_err_t unit_lorawan_log( uint8_t level );
/* @[declare_unit_lorawan_log] */

/** 
 * @brief Checks if the LoRaWAN unit is connected to the network.
 * 
 * @return [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
 *  - ESP_OK                : Success
 *  - ESP_FAIL              : Failed to check LoRaWAN connection state.
 */
/* @[declare_unit_lorawan_connected] */
esp_err_t unit_lorawan_connected( bool *state );
/* @[declare_unit_lorawan_connected] */

/** 
 * @brief Attempts to join the LoRaWAN network.
 * 
 * @return [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
 *  - ESP_OK                : Success
 *  - ESP_FAIL              : Failed attempt to join. Does not indicate a failed connection.
 */
/* @[declare_unit_lorawan_join] */
esp_err_t unit_lorawan_join( void );
/* @[declare_unit_lorawan_join] */

/** 
 * @brief Configures the LoRaWAN unit for OTTA.
 * 
 * @param devEUI The device EUI.
 * @param appEUI The application EUI.
 * @param appKey The application key.
 * @param mode The upload/download frequency mode. Can be DIFFERENT_FREQ_MODE or SAME_FREQ_MODE as @refunit_lorwan_uldlmode.
 * @return [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
 *  - ESP_OK                : Success
 */
/* @[declare_unit_lorawan_configOTTA] */
esp_err_t unit_lorawan_configOTTA( char *devEUI,char *appEUI, char *appKey, unit_lorwan_uldlmode mode );
/* @[declare_unit_lorawan_configOTTA] */

/** 
 * @brief Reboots the LoRaWAN unit.
 * 
 * @return [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
 *  - ESP_OK                : Success
 */
/* @[declare_unit_lorawan_reboot] */
esp_err_t unit_lorawan_reboot( void );
/* @[declare_unit_lorawan_reboot] */

/** 
 * @brief Checks if LoRaWAN unit peripheral is wired and communicating over UART.
 * 
 * @param state True if connected, false otherwise.
 * @return [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
 *  - ESP_OK                : Success
 *  - ESP_FAIL              : Failed to connect
 *  - ESP_ERR_INVALID_SIZE  : A device is connected but the response received was invalid
 */
/* @[declare_unit_lorawan_attached] */
esp_err_t unit_lorawan_attached( bool *state );
/* @[declare_unit_lorawan_attached] */

/**
 * @brief Sends uplink message.
 * 
 * Sends a uplink message to the LoRaWAN network. 
 * The message should be ASCII format and will get
 * converted to a hexidecimal string.
 * 
 * @param message The message to send.
 * @param length The length of the message.
 * 
 * @return [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
 * - ESP_OK                : Success
 * - ESP_FAIL              : Failed to send message
 * - ESP_ERR_INVALID_SIZE  : Message length is invalid
 * 
 * @note The message length after converting to hex should be less than 256 bytes.
 */
/* @[declare_unit_lorawan_send] */
esp_err_t unit_lorawan_send( char *message, size_t length );
/* @[declare_unit_lorawan_send] */

/** 
 * @brief Initializes the LoRaWAN driver over UART.
 * 
 * @return [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
 *  - ESP_OK                : Success
 *  - ESP_FAIL              : Failed to initialize
 */
/* @[declare_unit_lorawan_init] */
esp_err_t unit_lorawan_init( void );
/* @[declare_unit_lorawan_init] */

#ifdef __cplusplus
}
#endif
#endif