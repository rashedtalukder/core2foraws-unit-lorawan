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

#include "core2foraws.h"
#include "unit_lorawan.h"

#define UNIT_LORAWAN_DATA_RATE    115200

static const char *_TAG = "UNIT_LORAWAN";

esp_err_t unit_lorawan_check( bool *connected )
{
    ESP_LOGI( _TAG, "Checking LoRaWAN connection" );
    *connected = false;
    size_t written;
    esp_err_t err = core2foraws_expports_uart_write( "AT+CGMI?\r\n", 11, &written );

    ESP_LOGI( _TAG, "Wrote %d bytes", written );
    if( err == ESP_OK )
    {
        char response[32];
        size_t read;
        err = core2foraws_expports_uart_read( response, read );
        if( err == ESP_OK )
        {
            ESP_LOGI( _TAG, "Read %d bytes", read );
            ESP_LOGI( _TAG, "Response: %s", response );
            *connected = true;
        }
        else
        {
            ESP_LOGE( _TAG, "Failed to read response" );
        }
    }
    else
    {
        ESP_LOGE( _TAG, "LoRaWAN not connected" );
    }
    return err;
}

esp_err_t unit_lorawan_init( void )
{
    ESP_LOGI( _TAG, "Initializing LoRaWAN driver" );

    esp_err_t err = core2foraws_expports_uart_begin( UNIT_LORAWAN_DATA_RATE );
    if( err == ESP_OK )
    {
        ESP_LOGI( _TAG, "LoRaWAN driver initialized" );
    }
    else
    {
        ESP_LOGE( _TAG, "Failed to initialize LoRaWAN driver" );
    }
    return err;
}

