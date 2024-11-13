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
 * @brief Checks if LoRaWAN unit is connected over UART.
 * 
 * @param connected True if connected, false otherwise.
 * @return [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
 *  - ESP_OK                : Success
 */
esp_err_t unit_lorawan_connected( bool *connected );

/** 
 * @brief Initializes the LoRaWAN driver over UART.
 * 
 * @return [esp_err_t](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/system/esp_err.html#macros).
 *  - ESP_OK                : Success
 */
esp_err_t unit_lorawan_init( void );

#ifdef __cplusplus
}
#endif
#endif