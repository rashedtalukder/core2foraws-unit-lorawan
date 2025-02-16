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
#define UNIT_LORAWAN_MFG "ASR"
#define UNIT_LORAWAN_MODEL "6501"

static const char *_TAG = "UNIT_LORAWAN";

static esp_err_t _unit_lorawan_check_received_status( const char *cmd, size_t read, char *message );
static esp_err_t _unit_lorawan_send_cmd( char *cmd, size_t *wrote );
static esp_err_t _unit_lorawan_receive_cmd( char *response, size_t *read );

static const char *_unit_lorawan_uldlmode_str[2] = 
{
    "2",
    "1"
};

static esp_err_t _unit_lorawan_check_received_status( const char *cmd, size_t read, char *message )
{
    esp_err_t status = ESP_OK;

    if( read == 0 )
    {
        status = ESP_ERR_INVALID_SIZE;
    }
    else if( strnstr( message, "ERROR", strlen( message ) ) != NULL )
    {
        ESP_LOGE( _TAG, "Responded to command '%s' with error: %s", cmd, message );
        status = ESP_FAIL;
    }
    else
    {
        ESP_LOGD( _TAG, "Response for command '%s' OK", cmd );
    }

    return status;
}

static esp_err_t _unit_lorawan_send_cmd( char *cmd, size_t *wrote )
{
    uint8_t at_cmd_str_len = 3 + strlen( cmd ) + 2 + 1;
    char at_cmd_str[ at_cmd_str_len ];
    snprintf( at_cmd_str, at_cmd_str_len, "AT+%s\r\n", cmd );
    
    esp_err_t err = core2foraws_expports_uart_write( at_cmd_str, at_cmd_str_len, wrote );
    if( err == ESP_OK )
    {
        ESP_LOGI( _TAG, "Wrote %d bytes. Sent command: %s", *wrote, at_cmd_str );
    }
    else
    {
        ESP_LOGE( _TAG, "Failed to write command '%s'", at_cmd_str );
    }

    return err;
}

static esp_err_t _unit_lorawan_receive_cmd( char *response, size_t *read )
{
    esp_err_t err = core2foraws_expports_uart_read( ( unsigned char * )response, read );
    if( err == ESP_OK )
    {
        ESP_LOGI( _TAG, "Read %d bytes. Response: %s", *read, response );
    }
    else
    {
        ESP_LOGE( _TAG, "Failed to read response" );
    }    

    return err;
}

esp_err_t unit_lorawan_log( uint8_t level )
{
    ESP_LOGI( _TAG, "Setting LoRaWAN log level" );
    size_t written;
    char log_level_cmd[16];
    snprintf(log_level_cmd, sizeof(log_level_cmd), "ILOGLVL=%d", level);
    
    return _unit_lorawan_send_cmd( log_level_cmd, &written );
}

esp_err_t unit_lorawan_connected( bool *state )
{
    ESP_LOGI( _TAG, "Checking if LoRaWAN unit is connected" );
    *state = false;
    char *connection_station_cmd = "CSTATUS?";
    size_t written;

    esp_err_t err = _unit_lorawan_send_cmd( connection_station_cmd, &written );
    vTaskDelay( pdMS_TO_TICKS( 50 ) );
    
    if( err == ESP_OK )
    {
        char *response;
        response = ( char * ) malloc( 128 );
        size_t read;
        err = _unit_lorawan_receive_cmd( response, &read );
        if( err == ESP_OK )
        {
            err = _unit_lorawan_check_received_status( connection_station_cmd, read, ( char * )response );
            if ( err == ESP_OK )
            {
                // Check if the response contains either "+CSTATUS:04" or "+CSTATUS:08"
                if( strnstr( ( char * )response, "+CSTATUS:04", strlen( ( char * )response ) ) != NULL || 
                    strnstr( ( char * )response, "+CSTATUS:08", strlen( ( char * )response ) ) != NULL )
                {
                    *state = true;
                }
            }
        }
        else
        {
            ESP_LOGE( _TAG, "Failed to read response" );
        }

        free( response );
    }
    else
    {
        ESP_LOGE( _TAG, "LoRaWAN unit not connected" );
    }
    
    return err;
}

esp_err_t unit_lorawan_join( void )
{
    ESP_LOGI( _TAG, "Joining LoRaWAN network" );
    size_t written;
    char *join_cmd = "CJOIN=1,1,10,8";
    return _unit_lorawan_send_cmd( join_cmd, &written );
}

esp_err_t unit_lorawan_configOTTA( char *devEUI,char *appEUI, char *appKey, unit_lorwan_uldlmode mode )
{
    ESP_LOGI( _TAG, "Configuring LoRaWAN unit for OTTA" );
    size_t written;
    char *join_mode_cmd = "CJOINMODE=0";
    char *class_cmd = "CCLASS=2";
    char *work_mode_cmd = "CWORKMODE=2";
    char *rx_freq_mask_cmd = "CFREQBANDMASK=0001";
    char *rx_freq_window_cmd = "CRXP=0,0,923300000";
    char devEUI_cmd[32];
    snprintf(devEUI_cmd, sizeof(devEUI_cmd), "CDEVEUI=%s", devEUI);
    
    char appEUI_cmd[32];
    snprintf(appEUI_cmd, sizeof(appEUI_cmd), "CAPPEUI=%s", appEUI);
    
    char appKey_cmd[48];
    snprintf(appKey_cmd, sizeof(appKey_cmd), "CAPPKEY=%s", appKey);
    
    char uldl_mode_cmd[12];
    snprintf(uldl_mode_cmd, sizeof(uldl_mode_cmd), "CULDLMODE=%s", _unit_lorawan_uldlmode_str[mode]);

    esp_err_t err = _unit_lorawan_send_cmd( join_mode_cmd, &written );
    if ( err == ESP_OK )
    {
        err = _unit_lorawan_send_cmd( devEUI_cmd, &written );
        if( err == ESP_OK )
        {
            err = _unit_lorawan_send_cmd( appEUI_cmd, &written );
            if( err == ESP_OK )
            {
                err = _unit_lorawan_send_cmd( appKey_cmd, &written );
                if( err == ESP_OK )
                {
                    err = _unit_lorawan_send_cmd( uldl_mode_cmd, &written );
                    vTaskDelay( pdMS_TO_TICKS( 100 ) );
                    if( err == ESP_OK )
                    {
                        char *response;
                        response = ( char * ) malloc( 256 );
                        size_t read;
                        err = _unit_lorawan_receive_cmd( response, &read );
                        if( err == ESP_OK )
                        {
                            err = _unit_lorawan_check_received_status( "OTTA", read, ( char * )response );
                            if( err == ESP_OK )
                            {
                                ESP_LOGI( _TAG, "LoRaWAN unit configured credentials" );
                            }

                            memset( response, 0, read );
                        
                            err = _unit_lorawan_send_cmd( class_cmd, &written );
                            if( err == ESP_OK )
                            {
                                err = _unit_lorawan_send_cmd( work_mode_cmd, &written );
                                if( err == ESP_OK )
                                {
                                    err = _unit_lorawan_send_cmd( rx_freq_mask_cmd, &written );
                                    vTaskDelay( pdMS_TO_TICKS( 100 ) );  
                                    if( err == ESP_OK )
                                    {
                                        err = _unit_lorawan_send_cmd( rx_freq_window_cmd, &written );
                                        vTaskDelay( pdMS_TO_TICKS( 100 ) );  
                                        if( err == ESP_OK )
                                        {
                                            err = _unit_lorawan_receive_cmd( response, &read );
                                            if( err == ESP_OK )
                                            {
                                                err = _unit_lorawan_check_received_status( "", read, ( char * )response );
                                                if( err == ESP_OK )
                                                {
                                                    ESP_LOGI( _TAG, "LoRaWAN unit configured for OTTA" );
                                                }
                                            }
                                        }
                                        
                                    }
                                }                          
                            }
                        }
                        free( response );
                    }
                }
            }
        }
    }

    if( err != ESP_OK )
    {
        ESP_LOGE( _TAG, "Failed to configure LoRaWAN unit" );
    }

    return err;
}

esp_err_t unit_lorawan_reboot( void )
{
    ESP_LOGI( _TAG, "Rebooting LoRaWAN unit" );
    char *reset_cmd = "IREBOOT=0";
    size_t written;

    esp_err_t err = _unit_lorawan_send_cmd( reset_cmd, &written );
    vTaskDelay( pdMS_TO_TICKS( 50 ) );
    
    if( err == ESP_OK )
    {
       ESP_LOGI( _TAG, "LoRaWAN unit rebooted" );
    }
    else
    {
        ESP_LOGE( _TAG, "Failed to reboot LoRaWAN unit" );
    }

    return err;

}

esp_err_t unit_lorawan_attached( bool *state )
{
    ESP_LOGI( _TAG, "Checking if unit is attached" );
    *state = false;
    char *mfg_cmd = "CGMI?";
    size_t written;

    esp_err_t err = _unit_lorawan_send_cmd( mfg_cmd, &written );
    vTaskDelay( pdMS_TO_TICKS( 50 ) );
    
    if( err == ESP_OK )
    {
        char *response;
        response = ( char * ) malloc( 128 );
        size_t read;
        err = _unit_lorawan_receive_cmd( response, &read );
        if( err == ESP_OK )
        {            
            err = _unit_lorawan_check_received_status( mfg_cmd, read, ( char * )response );
            if ( err == ESP_OK )
            {
                if( strnstr( ( char * )response, UNIT_LORAWAN_MFG , strlen( ( char * )response ) ) != NULL )
                {
                    *state = true;
                }
            }
        }
        else
        {
            ESP_LOGE( _TAG, "Failed to read response" );
        }

        free( response );
    }
    else
    {
        ESP_LOGE( _TAG, "LoRaWAN unit not attached or available" );
    }
    
    return err;
}

esp_err_t unit_lorawan_send( char *message, size_t length )
{
    ESP_LOGI( _TAG, "Sending LoRaWAN message" );

    char *hex_message = ( char * ) malloc( length * 2 + 1 );
    for( size_t i = 0; i < length; i++ )
    {
        snprintf( hex_message + i * 2, 3, "%02X", message[ i ] );
    }

    size_t hex_message_len = strlen( hex_message );

    if( hex_message_len > 256 )
    {
        ESP_LOGE( _TAG, "Hexadecimal message length is too long to be sent. Got %d out of 256 max", hex_message_len );
        return ESP_ERR_INVALID_SIZE;
    }

    size_t written;
    char *send_cmd = "DTRX=1,8,8,";
    char *send_cmd_str = ( char * ) malloc( strlen( send_cmd ) + hex_message_len + 1 );
    snprintf( send_cmd_str, strlen( send_cmd ) + hex_message_len + 1, "%s%s", send_cmd, hex_message );
    
    esp_err_t err = _unit_lorawan_send_cmd( send_cmd_str, &written );
    free( hex_message );
    free( send_cmd_str );
    vTaskDelay( pdMS_TO_TICKS( 100 ) );
    
    if( err == ESP_OK )
    {
        char *response;
        response = ( char * ) malloc( 256 );
        size_t read;
        err = _unit_lorawan_receive_cmd( response, &read );
        if( err == ESP_OK )
        {
            err = _unit_lorawan_check_received_status( "SEND", read, ( char * )response );
            if ( err == ESP_OK )
            {
                ESP_LOGI( _TAG, "LoRaWAN message sent" );
            }
        }
        else
        {
            ESP_LOGE( _TAG, "Failed to read response" );
        }

        free( response );
    }
    else
    {
        ESP_LOGE( _TAG, "Failed to send LoRaWAN message" );
    }
    return err;
}

esp_err_t unit_lorawan_init( void )
{
    ESP_LOGI( _TAG, "Initializing LoRaWAN unit driver" );

    esp_err_t err = core2foraws_expports_uart_begin( UNIT_LORAWAN_DATA_RATE );
    bool plugged_in = false;
    err = unit_lorawan_attached( &plugged_in );
    if( err == ESP_OK && plugged_in )
    {
        bool flushed = false;
        core2foraws_expports_uart_read_flush( &flushed );
        err = unit_lorawan_log( 0 );
        if( err == ESP_OK )
        {
            size_t written;
            _unit_lorawan_send_cmd( "CSAVE", &written );
            unit_lorawan_reboot( );
            vTaskDelay( pdMS_TO_TICKS( 500 ) );
            ESP_LOGI( _TAG, "LoRaWAN unit driver initialized" );
        }
        else
        {
            ESP_LOGE( _TAG, "Failed to initialize LoRaWAN unit driver" );
        }
    }
    else
    {
        ESP_LOGE( _TAG, "LoRaWAN unit not attached or available" );
    }
    return err;
}

