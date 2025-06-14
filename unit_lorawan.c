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

#include "unit_lorawan.h"
#include "core2foraws.h"
#include "sdkconfig.h" // For CONFIG_* values

#define UNIT_LORAWAN_DATA_RATE            115200
#define UNIT_LORAWAN_MFG                  "ASR"
#define UNIT_LORAWAN_MODEL                "6501"
#define UNIT_LORAWAN_RESPONSE_TIMEOUT_MS  5000
#define UNIT_LORAWAN_COMMAND_DELAY_MS     100
#define UNIT_LORAWAN_MAX_RETRIES          3
#define UNIT_LORAWAN_RESPONSE_BUFFER_SIZE 512
#define UNIT_LORAWAN_HEX_MESSAGE_MAX_SIZE                                      \
  512 // Allows up to 256 bytes payload, doubled for hex encoding

// TTN US915 internal constants (not exposed to users)
#define TTN_US915_CLASS_DEFAULT     0 // Class A (most common)
#define TTN_US915_WORK_MODE_DEFAULT 2 // LoRaWAN mode

// ASR6501 frequency band masks for US915 sub-bands (internal use only)
#define ASR6501_US915_SUB_BAND_1_MASK "0001" // Channels 0-7
#define ASR6501_US915_SUB_BAND_2_MASK "0002" // Channels 8-15 (TTN)
#define ASR6501_US915_SUB_BAND_3_MASK "0004" // Channels 16-23
#define ASR6501_US915_SUB_BAND_4_MASK "0008" // Channels 24-31
#define ASR6501_US915_SUB_BAND_5_MASK "0010" // Channels 32-39
#define ASR6501_US915_SUB_BAND_6_MASK "0020" // Channels 40-47
#define ASR6501_US915_SUB_BAND_7_MASK "0040" // Channels 48-55
#define ASR6501_US915_SUB_BAND_8_MASK "0080" // Channels 56-63

// US915 data rate to maximum payload mapping for ASR6501
static const size_t us915_max_payload_sizes[] = {
    UNIT_LORAWAN_US915_MAX_PAYLOAD_DR0, // DR0: SF10, 125kHz
    UNIT_LORAWAN_US915_MAX_PAYLOAD_DR1, // DR1: SF9, 125kHz
    UNIT_LORAWAN_US915_MAX_PAYLOAD_DR2, // DR2: SF8, 125kHz
    UNIT_LORAWAN_US915_MAX_PAYLOAD_DR3, // DR3: SF7, 125kHz
    UNIT_LORAWAN_US915_MAX_PAYLOAD_DR4  // DR4: SF8, 500kHz
};

static const char *_TAG = "UNIT_LORAWAN";

// Enhanced response parsing structure
typedef struct
{
  bool success;
  char *response_data;
  size_t data_length;
  char error_code[ 16 ];
} lorawan_response_t;

static esp_err_t _unit_lorawan_send_at_command( const char *cmd,
                                                lorawan_response_t *response,
                                                uint32_t timeout_ms );
static esp_err_t
_unit_lorawan_parse_response( const char *raw_response, size_t response_len,
                              lorawan_response_t *parsed_response );
static esp_err_t _unit_lorawan_wait_for_response( char *buffer,
                                                  size_t buffer_size,
                                                  size_t *received_len,
                                                  uint32_t timeout_ms );
static esp_err_t _unit_lorawan_validate_payload_size( size_t payload_size,
                                                      uint8_t data_rate );
static void _unit_lorawan_cleanup_response( lorawan_response_t *response );
static const char *
_unit_lorawan_get_connection_status_description( const char *status_code );

static const char *_unit_lorawan_uldlmode_str[ 2 ] = { "2", "1" };

static const char *
_unit_lorawan_get_connection_status_description( const char *status_code )
{
  if( strstr( status_code, "04" ) )
  {
    return "Network joined successfully (OTAA)";
  }
  else if( strstr( status_code, "08" ) )
  {
    return "Network joined successfully (ABP)";
  }
  else if( strstr( status_code, "02" ) )
  {
    return "Network joining in progress";
  }
  else if( strstr( status_code, "01" ) )
  {
    return "Device not joined to network";
  }
  else if( strstr( status_code, "03" ) )
  {
    return "Network join failed";
  }
  return "Unknown connection status";
}

static esp_err_t _unit_lorawan_validate_payload_size( size_t payload_size,
                                                      uint8_t data_rate )
{
  size_t max_payload = 0;
  switch( data_rate )
  {
  case 0:
    max_payload = UNIT_LORAWAN_US915_MAX_PAYLOAD_DR0;
    break;
  case 1:
    max_payload = UNIT_LORAWAN_US915_MAX_PAYLOAD_DR1;
    break;
  case 2:
    max_payload = UNIT_LORAWAN_US915_MAX_PAYLOAD_DR2;
    break;
  case 3:
    max_payload = UNIT_LORAWAN_US915_MAX_PAYLOAD_DR3;
    break;
  case 4:
    max_payload = UNIT_LORAWAN_US915_MAX_PAYLOAD_DR4;
    break;
  default:
    ESP_LOGW( _TAG, "Unknown data rate %d, using maximum safe payload size",
              data_rate );
    max_payload = UNIT_LORAWAN_US915_MAX_PAYLOAD_DR0;
    break;
  }
  if( payload_size > max_payload )
  {
    ESP_LOGE( _TAG,
              "Payload size %zu exceeds maximum %zu bytes for data rate DR%d",
              payload_size, max_payload, data_rate );
    return ESP_ERR_INVALID_SIZE;
  }
  ESP_LOGD( _TAG, "Payload size %zu is valid for data rate DR%d (max: %zu)",
            payload_size, data_rate, max_payload );
  return ESP_OK;
}

static esp_err_t _unit_lorawan_wait_for_response( char *buffer,
                                                  size_t buffer_size,
                                                  size_t *received_len,
                                                  uint32_t timeout_ms )
{
  *received_len = 0;
  uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
  while( ( xTaskGetTickCount() * portTICK_PERIOD_MS - start_time ) <
         timeout_ms )
  {
    size_t available_bytes = 0;
    esp_err_t err =
        core2foraws_expports_uart_read( (uint8_t *)buffer, &available_bytes );
    if( err == ESP_OK && available_bytes > 0 )
    {
      *received_len = available_bytes;
      buffer[ available_bytes ] = '\0';
      ESP_LOGD( _TAG, "Received %zu bytes: %s", available_bytes, buffer );
      return ESP_OK;
    }
    vTaskDelay( pdMS_TO_TICKS( 50 ) );
  }
  ESP_LOGW( _TAG, "Timeout waiting for response after %u ms", timeout_ms );
  return ESP_ERR_TIMEOUT;
}

static esp_err_t
_unit_lorawan_parse_response( const char *raw_response, size_t response_len,
                              lorawan_response_t *parsed_response )
{
  if( !raw_response || !parsed_response || response_len == 0 )
  {
    return ESP_ERR_INVALID_ARG;
  }

  parsed_response->success = false;
  parsed_response->response_data = NULL;
  parsed_response->data_length = 0;
  memset( parsed_response->error_code, 0,
          sizeof( parsed_response->error_code ) );

  if( strstr( raw_response, "OK" ) != NULL )
  {
    parsed_response->success = true;
    ESP_LOGD( _TAG, "Command executed successfully" );

    if( strstr( raw_response, "+CGMI=" ) != NULL ||
        strstr( raw_response, "+CSTATUS:" ) != NULL ||
        strstr( raw_response, "+CDATARATE:" ) != NULL ||
        strstr( raw_response, "+CTXP:" ) != NULL ||
        strstr( raw_response, "+CRSSI:" ) != NULL ||
        strstr( raw_response, "+DTRX:" ) != NULL ||
        strstr( raw_response, "+CJOIN:" ) != NULL )
    {
      parsed_response->data_length = response_len;
      parsed_response->response_data = malloc( response_len + 1 );
      if( parsed_response->response_data )
      {
        memcpy( parsed_response->response_data, raw_response, response_len );
        parsed_response->response_data[ response_len ] = '\0';
        ESP_LOGD( _TAG, "Response data captured: %s",
                  parsed_response->response_data );
      }
    }
  }
  // Check for error responses
  else if( strstr( raw_response, "ERROR" ) != NULL )
  {
    parsed_response->success = false;
    // Try to extract error code
    char *error_pos = strstr( raw_response, "ERROR" );
    if( error_pos )
    {
      sscanf( error_pos, "ERROR:%15s", parsed_response->error_code );
    }
    ESP_LOGW( _TAG, "Command failed with error: %s",
              parsed_response->error_code );
  }
  // Check for specific LoRaWAN responses with +COMMAND: format (fallback for
  // responses without OK)
  else if( strstr( raw_response, "+CSTATUS:" ) != NULL ||
           strstr( raw_response, "+CGMI=" ) != NULL ||
           strstr( raw_response, "+CDATARATE:" ) != NULL ||
           strstr( raw_response, "+CTXP:" ) != NULL ||
           strstr( raw_response, "+CRSSI:" ) != NULL ||
           strstr( raw_response, "+DTRX:" ) != NULL ||
           strstr( raw_response, "+CJOIN:" ) != NULL )
  {
    parsed_response->success = true;
    parsed_response->data_length = response_len;
    parsed_response->response_data = malloc( response_len + 1 );
    if( parsed_response->response_data )
    {
      memcpy( parsed_response->response_data, raw_response, response_len );
      parsed_response->response_data[ response_len ] = '\0';
    }
  }

  return ESP_OK;
}

static esp_err_t _unit_lorawan_send_at_command( const char *cmd,
                                                lorawan_response_t *response,
                                                uint32_t timeout_ms )
{
  if( !cmd )
  {
    ESP_LOGE( _TAG, "Command cannot be NULL" );
    return ESP_ERR_INVALID_ARG;
  }

  // Format AT command
  size_t at_cmd_len =
      strlen( cmd ) + 6; // "AT+" + cmd + "\r\n" + null terminator
  char *at_cmd = malloc( at_cmd_len );
  if( !at_cmd )
  {
    ESP_LOGE( _TAG, "Failed to allocate memory for AT command" );
    return ESP_ERR_NO_MEM;
  }

  snprintf( at_cmd, at_cmd_len, "AT+%s\r\n", cmd );

  // Send command with retries
  esp_err_t err = ESP_FAIL;
  for( int retry = 0; retry < UNIT_LORAWAN_MAX_RETRIES; retry++ )
  {
    if( retry > 0 )
    {
      ESP_LOGW( _TAG, "Retrying command (attempt %d/%d): %s", retry + 1,
                UNIT_LORAWAN_MAX_RETRIES, cmd );
      vTaskDelay( pdMS_TO_TICKS( 500 ) ); // Wait before retry
    }

    // Clear any pending data
    bool flushed = false;
    core2foraws_expports_uart_read_flush( &flushed );

    // Send command
    size_t written = 0;
    err = core2foraws_expports_uart_write( at_cmd, strlen( at_cmd ), &written );
    if( err != ESP_OK )
    {
      ESP_LOGE( _TAG, "Failed to send AT command: %s", cmd );
      continue;
    }

    ESP_LOGD( _TAG, "Sent AT command (%zu bytes): %s", written, at_cmd );

    // Wait for command to be sent
    vTaskDelay( pdMS_TO_TICKS( UNIT_LORAWAN_COMMAND_DELAY_MS ) );

    // Read response
    char *response_buffer = malloc( UNIT_LORAWAN_RESPONSE_BUFFER_SIZE );
    if( !response_buffer )
    {
      ESP_LOGE( _TAG, "Failed to allocate response buffer" );
      err = ESP_ERR_NO_MEM;
      break;
    }

    size_t received_len = 0;
    err = _unit_lorawan_wait_for_response(
        response_buffer, UNIT_LORAWAN_RESPONSE_BUFFER_SIZE - 1, &received_len,
        timeout_ms );

    if( err == ESP_OK && response )
    {
      err = _unit_lorawan_parse_response( response_buffer, received_len,
                                          response );
    }

    free( response_buffer );

    if( err == ESP_OK )
    {
      break; // Success, exit retry loop
    }
  }

  free( at_cmd );

  if( err != ESP_OK )
  {
    ESP_LOGE( _TAG, "Command failed after %d retries: %s",
              UNIT_LORAWAN_MAX_RETRIES, cmd );
  }

  return err;
}

static void _unit_lorawan_cleanup_response( lorawan_response_t *response )
{
  if( response && response->response_data )
  {
    free( response->response_data );
    response->response_data = NULL;
    response->data_length = 0;
  }
}

esp_err_t unit_lorawan_log( uint8_t level )
{
  ESP_LOGI( _TAG, "Setting LoRaWAN log level to %d", level );

  if( level > UNIT_LORAWAN_LOG_LEVEL_MAX )
  {
    ESP_LOGW( _TAG, "Log level %d is out of range (0-%d), clamping to %d",
              level, UNIT_LORAWAN_LOG_LEVEL_MAX, UNIT_LORAWAN_LOG_LEVEL_MAX );
    level = UNIT_LORAWAN_LOG_LEVEL_MAX;
  }

  char log_cmd[ 32 ];
  snprintf( log_cmd, sizeof( log_cmd ), "ILOGLVL=%d", level );

  lorawan_response_t response = { 0 };
  esp_err_t err = _unit_lorawan_send_at_command(
      log_cmd, &response, UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );

  if( err == ESP_OK && response.success )
  {
    ESP_LOGI( _TAG, "✓ LoRaWAN log level set to %d successfully", level );
  }
  else
  {
    ESP_LOGE( _TAG, "✗ Failed to set LoRaWAN log level" );
  }

  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_connected( bool *state )
{
  if( !state )
  {
    ESP_LOGE( _TAG, "State parameter cannot be NULL" );
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGD( _TAG, "Checking LoRaWAN network connection status" );
  *state = false;

  lorawan_response_t response = { 0 };
  esp_err_t err = _unit_lorawan_send_at_command(
      "CSTATUS?", &response, UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );

  if( err == ESP_OK && response.success && response.response_data )
  {
    char *status_pos = strstr( response.response_data, "+CSTATUS:" );
    if( status_pos )
    {
      char status_code[ 8 ] = { 0 };
      if( sscanf( status_pos, "+CSTATUS:%7s", status_code ) == 1 )
      {
        if( strstr( status_code, "04" ) != NULL ||
            strstr( status_code, "08" ) != NULL )
        {
          *state = true;
          ESP_LOGI( _TAG, "✓ Device is connected to LoRaWAN network" );
        }
        else
        {
          const char *description =
              _unit_lorawan_get_connection_status_description( status_code );
          ESP_LOGI( _TAG, "✗ Device not connected to network - Status: %s (%s)",
                    status_code, description );
        }
      }
      else
      {
        ESP_LOGW( _TAG, "✗ Could not parse connection status from response" );
      }
    }
    else
    {
      ESP_LOGW( _TAG, "✗ Could not find +CSTATUS: in response: %s",
                response.response_data );
    }
  }
  else
  {
    ESP_LOGE( _TAG, "✗ Failed to check LoRaWAN connection status" );
  }

  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_join( void )
{
  ESP_LOGI( _TAG, "Attempting to join LoRaWAN network..." );

  lorawan_response_t response = { 0 };
  // Use longer timeout for join operation
  esp_err_t err =
      _unit_lorawan_send_at_command( "CJOIN=1,1,10,8", &response, 30000 );

  if( err == ESP_OK && response.success )
  {
    ESP_LOGI( _TAG, "✓ LoRaWAN join command sent successfully" );
    ESP_LOGI( _TAG,
              "  Join process initiated - this may take up to 30 seconds" );
    ESP_LOGI( _TAG, "  Use unit_lorawan_connected() to check join status" );
  }
  else
  {
    ESP_LOGE( _TAG, "✗ Failed to send LoRaWAN join command" );
  }

  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_configOTTA( char *devEUI, char *appEUI, char *appKey,
                                   unit_lorwan_uldlmode mode )
{
  if( !devEUI || !appEUI || !appKey )
  {
    ESP_LOGE( _TAG, "EUI and key parameters cannot be NULL" );
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI( _TAG,
            "Configuring LoRaWAN device for OTAA (Over-The-Air Activation)" );
  ESP_LOGI( _TAG, "  DevEUI: %s", devEUI );
  ESP_LOGI( _TAG, "  AppEUI: %s", appEUI );
  ESP_LOGI( _TAG, "  Mode: %s",
            ( mode == DIFFERENT_FREQ_MODE ) ? "Different frequency"
                                            : "Same frequency" );

  lorawan_response_t response = { 0 };
  esp_err_t err = ESP_OK;

  // Step 1: Set join mode to OTAA
  err = _unit_lorawan_send_at_command( "CJOINMODE=0", &response,
                                       UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );
  if( err != ESP_OK || !response.success )
  {
    ESP_LOGE( _TAG, "✗ Failed to set OTAA join mode" );
    goto cleanup;
  }
  ESP_LOGI( _TAG, "✓ OTAA join mode configured" );
  _unit_lorawan_cleanup_response( &response );

  // Step 2: Configure device EUI
  char deveui_cmd[ 64 ];
  snprintf( deveui_cmd, sizeof( deveui_cmd ), "CDEVEUI=%s", devEUI );
  err = _unit_lorawan_send_at_command( deveui_cmd, &response,
                                       UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );
  if( err != ESP_OK || !response.success )
  {
    ESP_LOGE( _TAG, "✗ Failed to configure Device EUI" );
    goto cleanup;
  }
  ESP_LOGI( _TAG, "✓ Device EUI configured" );
  _unit_lorawan_cleanup_response( &response );

  // Step 3: Configure application EUI
  char appeui_cmd[ 64 ];
  snprintf( appeui_cmd, sizeof( appeui_cmd ), "CAPPEUI=%s", appEUI );
  err = _unit_lorawan_send_at_command( appeui_cmd, &response,
                                       UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );
  if( err != ESP_OK || !response.success )
  {
    ESP_LOGE( _TAG, "✗ Failed to configure Application EUI" );
    goto cleanup;
  }
  ESP_LOGI( _TAG, "✓ Application EUI configured" );
  _unit_lorawan_cleanup_response( &response );

  // Step 4: Configure application key
  char appkey_cmd[ 80 ];
  snprintf( appkey_cmd, sizeof( appkey_cmd ), "CAPPKEY=%s", appKey );
  err = _unit_lorawan_send_at_command( appkey_cmd, &response,
                                       UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );
  if( err != ESP_OK || !response.success )
  {
    ESP_LOGE( _TAG, "✗ Failed to configure Application Key" );
    goto cleanup;
  }
  ESP_LOGI( _TAG, "✓ Application Key configured" );
  _unit_lorawan_cleanup_response( &response );

  // Step 5: Configure uplink/downlink mode
  char uldl_cmd[ 32 ];
  snprintf( uldl_cmd, sizeof( uldl_cmd ), "CULDLMODE=%s",
            _unit_lorawan_uldlmode_str[ mode ] );
  err = _unit_lorawan_send_at_command( uldl_cmd, &response,
                                       UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );
  if( err != ESP_OK || !response.success )
  {
    ESP_LOGE( _TAG, "✗ Failed to configure uplink/downlink mode" );
    goto cleanup;
  }
  ESP_LOGI( _TAG, "✓ Uplink/downlink mode configured" );
  _unit_lorawan_cleanup_response( &response );

  // Step 6: Set LoRaWAN class (Class A is most common)
  err = _unit_lorawan_send_at_command( "CCLASS=0", &response,
                                       UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );
  if( err != ESP_OK || !response.success )
  {
    ESP_LOGE( _TAG, "✗ Failed to set LoRaWAN class" );
    goto cleanup;
  }
  ESP_LOGI( _TAG, "✓ LoRaWAN Class A configured" );
  _unit_lorawan_cleanup_response( &response );

  // Step 7: Set work mode
  err = _unit_lorawan_send_at_command( "CWORKMODE=2", &response,
                                       UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );
  if( err != ESP_OK || !response.success )
  {
    ESP_LOGE( _TAG, "✗ Failed to set work mode" );
    goto cleanup;
  }
  ESP_LOGI( _TAG, "✓ Work mode configured" );
  _unit_lorawan_cleanup_response( &response );

  ESP_LOGI( _TAG, "✓ LoRaWAN device successfully configured for OTAA" );
  ESP_LOGI( _TAG, "  Note: Network-specific settings (frequency band, RX2, "
                  "etc.) may need to be configured separately" );

cleanup:
  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_reboot( void )
{
  ESP_LOGI( _TAG, "Rebooting LoRaWAN module..." );

  lorawan_response_t response = { 0 };
  esp_err_t err = _unit_lorawan_send_at_command(
      "IREBOOT=0", &response, UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );

  if( err == ESP_OK )
  {
    ESP_LOGI( _TAG, "✓ LoRaWAN module reboot command sent" );
    ESP_LOGI( _TAG, "  Waiting for module to restart..." );
    vTaskDelay( pdMS_TO_TICKS( 2000 ) ); // Give module time to reboot
  }
  else
  {
    ESP_LOGE( _TAG, "✗ Failed to reboot LoRaWAN module" );
  }

  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_attached( bool *state )
{
  if( !state )
  {
    ESP_LOGE( _TAG, "State parameter cannot be NULL" );
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGD( _TAG, "Checking if LoRaWAN module is attached and responding" );
  *state = false;

  lorawan_response_t response = { 0 };
  esp_err_t err = _unit_lorawan_send_at_command(
      "CGMI?", &response, UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );

  if( err == ESP_OK && response.success && response.response_data )
  {
    // Parse manufacturer from +CGMI=XXX format
    char *cgmi_pos = strstr( response.response_data, "+CGMI=" );
    if( cgmi_pos )
    {
      char manufacturer[ 16 ] = { 0 };
      if( sscanf( cgmi_pos, "+CGMI=%15s", manufacturer ) == 1 )
      {
        if( strstr( manufacturer, UNIT_LORAWAN_MFG ) != NULL )
        {
          *state = true;
          ESP_LOGI( _TAG, "✓ LoRaWAN module detected and responding" );
          ESP_LOGD( _TAG, "  Module manufacturer: %s", manufacturer );
        }
        else
        {
          ESP_LOGW( _TAG, "✗ Unknown module detected - Expected: %s, Got: %s",
                    UNIT_LORAWAN_MFG, manufacturer );
        }
      }
      else
      {
        ESP_LOGW( _TAG, "✗ Could not parse manufacturer from response" );
      }
    }
    else
    {
      ESP_LOGW( _TAG, "✗ Could not find +CGMI= in response: %s",
                response.response_data );
    }
  }
  else
  {
    ESP_LOGE( _TAG, "✗ No LoRaWAN module detected or not responding" );
    ESP_LOGE( _TAG, "  Check physical connections and power" );
  }

  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_send( char *message, size_t length )
{
  if( !message || length == 0 )
  {
    ESP_LOGE( _TAG, "Message cannot be NULL or empty" );
    return ESP_ERR_INVALID_ARG;
  }

  // Get current data rate for proper payload validation
  uint8_t current_dr;
  size_t max_payload;
  esp_err_t dr_err =
      unit_lorawan_get_data_rate_info( &current_dr, &max_payload );
  if( dr_err != ESP_OK )
  {
    ESP_LOGW(
        _TAG,
        "Failed to get current data rate, using conservative validation" );
    current_dr = 0;
    max_payload = UNIT_LORAWAN_US915_MAX_PAYLOAD_DR0;
  }

  // Validate payload size against current data rate
  esp_err_t size_check =
      _unit_lorawan_validate_payload_size( length, current_dr );
  if( size_check != ESP_OK )
  {
    return size_check;
  }

  ESP_LOGI( _TAG, "Sending LoRaWAN message (%zu bytes) on DR%d", length,
            current_dr );
  ESP_LOGD( _TAG, "  Message content: %.*s", (int)length, message );

  // Convert message to hexadecimal
  char *hex_message = malloc( length * 2 + 1 );
  if( !hex_message )
  {
    ESP_LOGE( _TAG, "Failed to allocate memory for hex conversion" );
    return ESP_ERR_NO_MEM;
  }

  for( size_t i = 0; i < length; i++ )
  {
    snprintf( hex_message + i * 2, 3, "%02X", (uint8_t)message[ i ] );
  }
  hex_message[ length * 2 ] = '\0';

  ESP_LOGD( _TAG, "  Hex encoded: %s", hex_message );

  // Check hex message size limit
  size_t hex_len = strlen( hex_message );
  if( hex_len > UNIT_LORAWAN_HEX_MESSAGE_MAX_SIZE )
  {
    ESP_LOGE( _TAG, "Hex message too long: %zu bytes (max: %d)", hex_len,
              UNIT_LORAWAN_HEX_MESSAGE_MAX_SIZE );
    free( hex_message );
    return ESP_ERR_INVALID_SIZE;
  }

  char *send_cmd = malloc( strlen( "DTRX=1,2," ) + 10 + hex_len +
                           1 ); // Extra space for length
  if( !send_cmd )
  {
    ESP_LOGE( _TAG, "Failed to allocate memory for send command" );
    free( hex_message );
    return ESP_ERR_NO_MEM;
  }

  // Use confirmed message (1), 2 retries, with hex payload
  snprintf( send_cmd, strlen( "DTRX=1,2," ) + 10 + hex_len + 1,
            "DTRX=1,2,%zu,%s", length, hex_message );

  lorawan_response_t response = { 0 };
  esp_err_t err = _unit_lorawan_send_at_command(
      send_cmd, &response, 30000 ); // Longer timeout for send

  if( err == ESP_OK && response.success )
  {
    ESP_LOGI( _TAG, "✓ LoRaWAN message sent successfully" );
    ESP_LOGI( _TAG,
              "  Message will be transmitted when network conditions allow" );

    if( response.response_data )
    {
      if( strstr( response.response_data, "OK+SEND:" ) )
      {
        ESP_LOGI( _TAG, "  Message queued for transmission" );
      }
      if( strstr( response.response_data, "OK+SENT:" ) )
      {
        ESP_LOGI( _TAG, "  Message transmitted to network" );
      }
      if( strstr( response.response_data, "OK+RECV:" ) )
      {
        ESP_LOGI( _TAG, "  Network acknowledgment received" );
      }
      if( strstr( response.response_data, "ERR+SEND:" ) )
      {
        ESP_LOGW( _TAG, "  Message send error detected" );
      }
    }
  }
  else
  {
    ESP_LOGE( _TAG, "✗ Failed to send LoRaWAN message" );
    if( response.response_data )
    {
      ESP_LOGE( _TAG, "  Error details: %s", response.response_data );
    }
  }

  free( hex_message );
  free( send_cmd );
  _unit_lorawan_cleanup_response( &response );
  return err;
}

static esp_err_t
_configure_ttn_network_parameters( const unit_lorawan_ttn_config_t *config )
{
  ESP_LOGI( _TAG, "Configuring TTN-specific network parameters" );

  lorawan_response_t response = { 0 };
  esp_err_t err = ESP_OK;

  // Note: ASR6501 doesn't support manual RX2 configuration via AT commands
  // RX2 parameters are automatically handled by the LoRaWAN stack per regional
  // parameters
  ESP_LOGI(
      _TAG,
      "RX2 parameters automatically configured per US915 regional parameters" );

  // Enable/disable Adaptive Data Rate
  char adr_cmd[ 16 ];
  snprintf( adr_cmd, sizeof( adr_cmd ), "CADR=%d",
            config->adr_enabled ? 1 : 0 );

  err = _unit_lorawan_send_at_command( adr_cmd, &response,
                                       UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );
  if( err != ESP_OK || !response.success )
  {
    ESP_LOGE( _TAG, "Failed to configure ADR setting" );
    goto cleanup;
  }
  ESP_LOGI( _TAG, "✓ Adaptive Data Rate %s",
            config->adr_enabled ? "enabled" : "disabled" );
  _unit_lorawan_cleanup_response( &response );

  char dr_cmd[ 32 ];
  snprintf( dr_cmd, sizeof( dr_cmd ), "CDATARATE=%d", config->data_rate );

  err = _unit_lorawan_send_at_command( dr_cmd, &response,
                                       UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );
  if( err != ESP_OK || !response.success )
  {
    ESP_LOGW(
        _TAG,
        "Initial data rate setting failed - will rely on ADR if enabled" );
    ESP_LOGI( _TAG, "✓ Network will use default data rate (ADR: %s)",
              config->adr_enabled ? "enabled" : "disabled" );
    _unit_lorawan_cleanup_response( &response );
    err = ESP_OK; // Don't fail configuration for this
    goto cleanup;
  }
  ESP_LOGI( _TAG,
            "✓ Initial data rate configured (DR%d, max payload: %zu bytes)",
            config->data_rate, us915_max_payload_sizes[ config->data_rate ] );
  _unit_lorawan_cleanup_response( &response );

cleanup:
  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_set_rx2_frequency( uint32_t frequency )
{
  ESP_LOGW( _TAG, "RX2 frequency configuration not supported by ASR6501" );
  ESP_LOGW( _TAG, "Device uses automatic RX2 settings per LoRaWAN US915 "
                  "regional parameters" );
  ESP_LOGW(
      _TAG,
      "TTN US915 RX2: 923.3 MHz, DR8 (configured automatically by stack)" );
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t unit_lorawan_set_rx2_data_rate( uint8_t data_rate )
{
  ESP_LOGW( _TAG, "RX2 data rate configuration not supported by ASR6501" );
  ESP_LOGW( _TAG, "Device uses automatic RX2 settings per LoRaWAN US915 "
                  "regional parameters" );
  ESP_LOGW(
      _TAG,
      "TTN US915 RX2: 923.3 MHz, DR8 (configured automatically by stack)" );
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t unit_lorawan_get_channel_rssi( uint8_t freq_band_idx,
                                         int16_t *rssi_values,
                                         size_t *channel_count )
{
  if( !rssi_values || !channel_count )
  {
    ESP_LOGE( _TAG, "RSSI values and channel count parameters cannot be NULL" );
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI( _TAG, "Scanning RSSI for frequency band %d", freq_band_idx );

  char cmd[ 32 ];
  snprintf( cmd, sizeof( cmd ), "CRSSI %d?", freq_band_idx );

  lorawan_response_t response = { 0 };
  esp_err_t err = _unit_lorawan_send_at_command(
      cmd, &response, UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );

  if( err == ESP_OK && response.success && response.response_data )
  {
    // Response format: +CRSSI:\n0:<rssi>\n1:<rssi>\n...\n7:<rssi>\nOK
    char *line_start = strstr( response.response_data, "+CRSSI:" );
    if( line_start )
    {
      line_start = strchr( line_start, '\n' ); // Skip to first data line
      if( line_start )
        line_start++; // Move past newline

      size_t count = 0;
      char *line = line_start;

      // Parse all 8 channels (0-7)
      while( line && count < 8 )
      {
        int channel, rssi_val;
        if( sscanf( line, "%d:%d", &channel, &rssi_val ) == 2 )
        {
          if( channel >= 0 && channel <= 7 && channel == (int)count )
          {
            rssi_values[ count ] = (int16_t)rssi_val;
            count++;
            ESP_LOGD( _TAG, "Channel %d: %d dBm", channel, rssi_val );
          }
          else
          {
            ESP_LOGW( _TAG, "Unexpected channel number %d at position %zu",
                      channel, count );
          }
        }
        else
        {
          ESP_LOGW( _TAG, "Failed to parse RSSI line: %s", line );
        }

        line = strchr( line, '\n' );
        if( line )
          line++; // Move to next line
      }

      if( count == 8 )
      {
        *channel_count = count;
        ESP_LOGI( _TAG,
                  "✓ Successfully scanned RSSI for all %zu channels in "
                  "frequency band %d",
                  count, freq_band_idx );
      }
      else
      {
        ESP_LOGW( _TAG, "Expected 8 channels but only parsed %zu", count );
        *channel_count = count;
      }
    }
    else
    {
      ESP_LOGE( _TAG, "Invalid RSSI response format - missing +CRSSI:" );
      err = ESP_FAIL;
    }
  }
  else
  {
    ESP_LOGE( _TAG,
              "✗ Failed to scan channel RSSI values for frequency band %d",
              freq_band_idx );
  }

  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_set_data_rate( uint8_t data_rate )
{
  if( data_rate > UNIT_LORAWAN_US915_DATA_RATE_MAX )
  {
    ESP_LOGE( _TAG, "Invalid data rate %d for US915 (valid range: %d-%d)",
              data_rate, UNIT_LORAWAN_US915_DATA_RATE_MIN,
              UNIT_LORAWAN_US915_DATA_RATE_MAX );
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI( _TAG, "Setting data rate to DR%d (max payload: %zu bytes)",
            data_rate, us915_max_payload_sizes[ data_rate ] );

  char dr_cmd[ 32 ];
  snprintf( dr_cmd, sizeof( dr_cmd ), "CDATARATE=%d", data_rate );

  lorawan_response_t response = { 0 };
  esp_err_t err = _unit_lorawan_send_at_command(
      dr_cmd, &response, UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );

  if( err == ESP_OK && response.success )
  {
    ESP_LOGI( _TAG, "✓ Data rate set to DR%d successfully", data_rate );
  }
  else
  {
    ESP_LOGE( _TAG, "✗ Failed to set data rate to DR%d", data_rate );
    ESP_LOGW( _TAG,
              "Note: If ADR is enabled, network may override this setting" );
  }

  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_set_retries( uint8_t message_type, uint8_t retries )
{
  if( message_type > 1 )
  {
    ESP_LOGE( _TAG, "Invalid message type %d (0=unconfirmed, 1=confirmed)",
              message_type );
    return ESP_ERR_INVALID_ARG;
  }

  if( retries < 1 || retries > 15 )
  {
    ESP_LOGE( _TAG, "Invalid retry count %d (valid range: 1-15)", retries );
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI( _TAG, "Setting %s message retries to %d",
            message_type ? "confirmed" : "unconfirmed", retries );

  char cmd[ 32 ];
  snprintf( cmd, sizeof( cmd ), "CNBTRIALS=%d,%d", message_type, retries );

  lorawan_response_t response = { 0 };
  esp_err_t err = _unit_lorawan_send_at_command(
      cmd, &response, UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );

  if( err == ESP_OK && response.success )
  {
    ESP_LOGI( _TAG, "✓ Retry count set successfully" );
  }
  else
  {
    ESP_LOGE( _TAG, "✗ Failed to set retry count" );
  }

  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_set_tx_power( uint8_t power_index )
{
  if( power_index > 7 )
  {
    ESP_LOGE( _TAG, "Invalid TX power index %d (valid range: 0-7)",
              power_index );
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI( _TAG, "Setting TX power to index %d", power_index );

  char cmd[ 16 ];
  snprintf( cmd, sizeof( cmd ), "CTXP=%d", power_index );

  lorawan_response_t response = { 0 };
  esp_err_t err = _unit_lorawan_send_at_command(
      cmd, &response, UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );

  if( err == ESP_OK && response.success )
  {
    ESP_LOGI( _TAG, "✓ TX power set successfully" );
  }
  else
  {
    ESP_LOGE( _TAG, "✗ Failed to set TX power" );
  }

  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_get_tx_power( uint8_t *power_index )
{
  if( !power_index )
  {
    ESP_LOGE( _TAG, "TX power parameter cannot be NULL" );
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGD( _TAG, "Getting current TX power" );

  lorawan_response_t response = { 0 };
  esp_err_t err = _unit_lorawan_send_at_command(
      "CTXP?", &response, UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );

  if( err == ESP_OK && response.success && response.response_data )
  {
    char *txp_pos = strstr( response.response_data, "+CTXP:" );
    if( txp_pos )
    {
      int power_value = 0;
      if( sscanf( txp_pos, "+CTXP:%d", &power_value ) == 1 )
      {
        *power_index = (uint8_t)power_value;
        ESP_LOGI( _TAG, "Current TX power index: %d", *power_index );
      }
      else
      {
        ESP_LOGE( _TAG, "Failed to parse TX power from response" );
        err = ESP_FAIL;
      }
    }
    else
    {
      ESP_LOGE( _TAG, "TX power data not found in response" );
      err = ESP_FAIL;
    }
  }
  else
  {
    ESP_LOGE( _TAG, "Failed to get TX power" );
  }

  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_link_check( uint8_t mode )
{
  if( mode > 2 )
  {
    ESP_LOGE(
        _TAG,
        "Invalid link check mode %d (0=disable, 1=once, 2=after every uplink)",
        mode );
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI( _TAG, "Setting link check mode to %d", mode );

  char cmd[ 16 ];
  snprintf( cmd, sizeof( cmd ), "CLINKCHECK=%d", mode );

  lorawan_response_t response = { 0 };
  esp_err_t err = _unit_lorawan_send_at_command(
      cmd, &response, mode == 1 ? 30000 : UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );

  if( err == ESP_OK && response.success )
  {
    ESP_LOGI( _TAG, "✓ Link check configured successfully" );

    if( mode == 1 && response.response_data )
    {
      char *link_result = strstr( response.response_data, "+CLINKCHECK:" );
      if( link_result )
      {
        int result, margin, gateways, rssi, snr;
        if( sscanf( link_result, "+CLINKCHECK:%d,%d,%d,%d,%d", &result, &margin,
                    &gateways, &rssi, &snr ) == 5 )
        {
          if( result == 0 )
          {
            ESP_LOGI( _TAG, "✓ Link check successful:" );
            ESP_LOGI( _TAG, "  Demod Margin: %d", margin );
            ESP_LOGI( _TAG, "  Gateways: %d", gateways );
            ESP_LOGI( _TAG, "  RSSI: %d dBm", rssi );
            ESP_LOGI( _TAG, "  SNR: %d", snr );
          }
          else
          {
            ESP_LOGW( _TAG, "✗ Link check failed with result code %d", result );
          }
        }
      }
    }
  }
  else
  {
    ESP_LOGE( _TAG, "✗ Failed to configure link check" );
  }

  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_save_config( void )
{
  ESP_LOGI( _TAG, "Saving LoRaWAN configuration to non-volatile memory" );

  lorawan_response_t response = { 0 };
  esp_err_t err = _unit_lorawan_send_at_command(
      "CSAVE", &response, UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );

  if( err == ESP_OK && response.success )
  {
    ESP_LOGI( _TAG, "✓ Configuration saved successfully" );
  }
  else
  {
    ESP_LOGE( _TAG, "✗ Failed to save configuration" );
  }

  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_restore_defaults( void )
{
  ESP_LOGI( _TAG, "Restoring LoRaWAN factory default configuration" );

  lorawan_response_t response = { 0 };
  esp_err_t err = _unit_lorawan_send_at_command(
      "CRESTORE", &response, UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );

  if( err == ESP_OK && response.success )
  {
    ESP_LOGI( _TAG, "✓ Factory defaults restored successfully" );
    ESP_LOGI( _TAG, "  Note: You may need to reboot the module for changes to "
                    "take effect" );
  }
  else
  {
    ESP_LOGE( _TAG, "✗ Failed to restore factory defaults" );
  }

  _unit_lorawan_cleanup_response( &response );
  return err;
}

esp_err_t unit_lorawan_send_raw_command( const char *command, char *response,
                                         size_t response_size,
                                         uint32_t timeout_ms )
{
  if( !command )
  {
    ESP_LOGE( _TAG, "Command cannot be NULL" );
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI( _TAG, "Sending raw AT command: %s", command );

  lorawan_response_t lorawan_resp = { 0 };
  esp_err_t err =
      _unit_lorawan_send_at_command( command, &lorawan_resp, timeout_ms );

  if( err == ESP_OK && response && response_size > 0 )
  {
    // Copy response data if buffer provided
    if( lorawan_resp.response_data )
    {
      strncpy( response, lorawan_resp.response_data, response_size - 1 );
      response[ response_size - 1 ] = '\0';
    }
    else
    {
      response[ 0 ] = '\0';
    }
  }

  _unit_lorawan_cleanup_response( &lorawan_resp );
  return err;
}

esp_err_t unit_lorawan_init( void )
{
  ESP_LOGI( _TAG, "Initializing LoRaWAN module driver" );

  // Initialize UART for LoRaWAN communication
  esp_err_t err = core2foraws_expports_uart_begin( UNIT_LORAWAN_DATA_RATE );
  if( err != ESP_OK )
  {
    ESP_LOGE( _TAG, "✗ Failed to initialize UART for LoRaWAN communication" );
    return err;
  }
  ESP_LOGI( _TAG, "✓ UART initialized at %d baud", UNIT_LORAWAN_DATA_RATE );

  // Clear any pending data in UART buffer
  bool flushed = false;
  core2foraws_expports_uart_read_flush( &flushed );
  if( flushed )
  {
    ESP_LOGD( _TAG, "✓ UART buffer cleared" );
  }

  // Check if LoRaWAN module is connected
  bool attached = false;
  err = unit_lorawan_attached( &attached );
  if( err != ESP_OK || !attached )
  {
    ESP_LOGE( _TAG, "✗ LoRaWAN module not detected" );
    ESP_LOGE( _TAG, "  Please check:" );
    ESP_LOGE( _TAG, "  - Module is properly connected to expansion port C" );
    ESP_LOGE( _TAG, "  - Module has power (check LED indicators)" );
    ESP_LOGE( _TAG, "  - UART connections are secure" );
    return ESP_FAIL;
  }

  // Set log level to minimal for cleaner operation
  err = unit_lorawan_log( 1 ); // Error level only
  if( err != ESP_OK )
  {
    ESP_LOGW( _TAG, "⚠ Failed to set log level, continuing anyway" );
  }

  // Save configuration and reboot module
  lorawan_response_t response = { 0 };
  err = _unit_lorawan_send_at_command( "CSAVE", &response,
                                       UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );
  if( err == ESP_OK && response.success )
  {
    ESP_LOGI( _TAG, "✓ Configuration saved to module" );
  }
  else
  {
    ESP_LOGW( _TAG, "⚠ Failed to save configuration" );
  }
  _unit_lorawan_cleanup_response( &response );

  // Reboot module to ensure clean state
  unit_lorawan_reboot();

  ESP_LOGI( _TAG, "✓ LoRaWAN module driver initialized successfully" );
  ESP_LOGI( _TAG, "  Ready for OTAA configuration and network joining" );

  return ESP_OK;
}

// TTN US915 configuration functions

static const char *_get_us915_sub_band_mask( uint8_t sub_band )
{
  switch( sub_band )
  {
  case 1:
    return ASR6501_US915_SUB_BAND_1_MASK;
  case 2:
    return ASR6501_US915_SUB_BAND_2_MASK;
  case 3:
    return ASR6501_US915_SUB_BAND_3_MASK;
  case 4:
    return ASR6501_US915_SUB_BAND_4_MASK;
  case 5:
    return ASR6501_US915_SUB_BAND_5_MASK;
  case 6:
    return ASR6501_US915_SUB_BAND_6_MASK;
  case 7:
    return ASR6501_US915_SUB_BAND_7_MASK;
  case 8:
    return ASR6501_US915_SUB_BAND_8_MASK;
  default:
    return ASR6501_US915_SUB_BAND_2_MASK; // Default to TTN sub-band
  }
}

static esp_err_t _validate_ttn_config( const unit_lorawan_ttn_config_t *config )
{
  if( !config )
  {
    ESP_LOGE( _TAG, "TTN configuration cannot be NULL" );
    return ESP_ERR_INVALID_ARG;
  }

  // Validate DevEUI (16 hex characters)
  if( strlen( config->dev_eui ) != UNIT_LORAWAN_EUI_LENGTH )
  {
    ESP_LOGE( _TAG, "Invalid DevEUI length: %zu (expected %d)",
              strlen( config->dev_eui ), UNIT_LORAWAN_EUI_LENGTH );
    return ESP_ERR_INVALID_ARG;
  }

  // Validate AppEUI (16 hex characters)
  if( strlen( config->app_eui ) != UNIT_LORAWAN_EUI_LENGTH )
  {
    ESP_LOGE( _TAG, "Invalid AppEUI length: %zu (expected %d)",
              strlen( config->app_eui ), UNIT_LORAWAN_EUI_LENGTH );
    return ESP_ERR_INVALID_ARG;
  }

  // Validate AppKey (32 hex characters)
  if( strlen( config->app_key ) != UNIT_LORAWAN_APP_KEY_LENGTH )
  {
    ESP_LOGE( _TAG, "Invalid AppKey length: %zu (expected %d)",
              strlen( config->app_key ), UNIT_LORAWAN_APP_KEY_LENGTH );
    return ESP_ERR_INVALID_ARG;
  }

  // Validate sub-band range (1-8 for US915)
  if( config->sub_band < UNIT_LORAWAN_US915_SUB_BAND_MIN ||
      config->sub_band > UNIT_LORAWAN_US915_SUB_BAND_MAX )
  {
    ESP_LOGE( _TAG, "Invalid US915 sub-band: %d (valid range: %d-%d)",
              config->sub_band, UNIT_LORAWAN_US915_SUB_BAND_MIN,
              UNIT_LORAWAN_US915_SUB_BAND_MAX );
    return ESP_ERR_INVALID_ARG;
  }

  // Validate data rate for US915 (0-4)
  if( config->data_rate > UNIT_LORAWAN_US915_DATA_RATE_MAX )
  {
    ESP_LOGE( _TAG, "Invalid US915 data rate: %d (valid range: %d-%d)",
              config->data_rate, UNIT_LORAWAN_US915_DATA_RATE_MIN,
              UNIT_LORAWAN_US915_DATA_RATE_MAX );
    return ESP_ERR_INVALID_ARG;
  }

  // Validate RX2 data rate (typically 8 for TTN US915)
  if( config->rx2_data_rate > 15 )
  {
    ESP_LOGE( _TAG, "Invalid RX2 data rate: %d (valid range: 0-15)",
              config->rx2_data_rate );
    return ESP_ERR_INVALID_ARG;
  }

  return ESP_OK;
}

static esp_err_t _configure_us915_frequency_plan( uint8_t sub_band )
{
  ESP_LOGI( _TAG, "Configuring US915 frequency plan (sub-band %d)", sub_band );

  lorawan_response_t response = { 0 };
  esp_err_t err = ESP_OK;

  // Set frequency band to US915
  err = _unit_lorawan_send_at_command( "CFREQBANDMASK=0001", &response,
                                       UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );
  if( err != ESP_OK || !response.success )
  {
    ESP_LOGE( _TAG, "Failed to set US915 frequency band" );
    goto cleanup;
  }
  ESP_LOGD( _TAG, "✓ US915 frequency band set" );
  _unit_lorawan_cleanup_response( &response );

  // Configure channel mask for specific sub-band
  const char *sub_band_mask = _get_us915_sub_band_mask( sub_band );
  char channel_cmd[ 32 ];
  snprintf( channel_cmd, sizeof( channel_cmd ), "CFREQBANDMASK=%s",
            sub_band_mask );

  err = _unit_lorawan_send_at_command( channel_cmd, &response,
                                       UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );
  if( err != ESP_OK || !response.success )
  {
    ESP_LOGE( _TAG, "Failed to configure US915 sub-band %d", sub_band );
    goto cleanup;
  }
  ESP_LOGI( _TAG, "✓ US915 sub-band %d configured (channels %d-%d)", sub_band,
            ( sub_band - 1 ) * 8, ( sub_band - 1 ) * 8 + 7 );
  _unit_lorawan_cleanup_response( &response );

cleanup:
  _unit_lorawan_cleanup_response( &response );
  return err;
}

static void ttn_join_monitor_task( void *pvParameters )
{
  typedef struct
  {
    unit_lorawan_ttn_join_callback_t callback;
    void *user_data;
    uint16_t timeout_sec;
  } join_monitor_params_t;

  join_monitor_params_t *monitor_params = (join_monitor_params_t *)pvParameters;
  bool connected = false;
  uint16_t elapsed_sec = 0;

  ESP_LOGI( _TAG, "TTN join monitoring started (timeout: %d seconds)",
            monitor_params->timeout_sec );

  while( elapsed_sec < monitor_params->timeout_sec )
  {
    vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    elapsed_sec++;

    esp_err_t check_err = unit_lorawan_connected( &connected );
    if( check_err == ESP_OK && connected )
    {
      ESP_LOGI( _TAG, "✓ TTN network join successful after %d seconds",
                elapsed_sec );
      monitor_params->callback( true, 0, monitor_params->user_data );
      break;
    }

    // Log progress every 10 seconds
    if( elapsed_sec % 10 == 0 )
    {
      ESP_LOGI( _TAG, "TTN join in progress... (%d/%d seconds)", elapsed_sec,
                monitor_params->timeout_sec );
    }
  }

  if( !connected )
  {
    ESP_LOGE( _TAG, "✗ TTN network join timeout after %d seconds",
              monitor_params->timeout_sec );
    ESP_LOGE( _TAG, "Check TTN console, gateway coverage, and credentials" );
    monitor_params->callback( false, 1,
                              monitor_params->user_data ); // Timeout error
  }

  free( monitor_params );
  vTaskDelete( NULL );
}

esp_err_t unit_lorawan_configure_ttn_us915(
    const unit_lorawan_ttn_config_t *config,
    unit_lorawan_ttn_join_callback_t join_callback, void *user_data )
{
  ESP_LOGI( _TAG, "Configuring LoRaWAN for The Things Network (TTN) US915" );

  // Validate configuration
  esp_err_t err = _validate_ttn_config( config );
  if( err != ESP_OK )
  {
    return err;
  }

  // Check if module is attached and responding
  bool attached = false;
  err = unit_lorawan_attached( &attached );
  if( err != ESP_OK || !attached )
  {
    ESP_LOGE( _TAG, "LoRaWAN module not detected or not responding" );
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI( _TAG, "TTN US915 Configuration:" );
  ESP_LOGI( _TAG, "  DevEUI: %s", config->dev_eui );
  ESP_LOGI( _TAG, "  AppEUI: %s", config->app_eui );
  ESP_LOGI( _TAG, "  Sub-band: %d (channels %d-%d)", config->sub_band,
            ( config->sub_band - 1 ) * 8, ( config->sub_band - 1 ) * 8 + 7 );
  ESP_LOGI( _TAG, "  Data Rate: DR%d (max %zu bytes payload)",
            config->data_rate, us915_max_payload_sizes[ config->data_rate ] );
  ESP_LOGI( _TAG, "  ADR: %s", config->adr_enabled ? "Enabled" : "Disabled" );
  ESP_LOGI( _TAG, "  RX2: %u Hz, DR%d (configured automatically by stack)",
            config->rx2_frequency, config->rx2_data_rate );

  // Configure US915 frequency plan
  err = _configure_us915_frequency_plan( config->sub_band );
  if( err != ESP_OK )
  {
    ESP_LOGE( _TAG, "Failed to configure US915 frequency plan" );
    return err;
  }

  // Configure OTAA credentials
  err =
      unit_lorawan_configOTTA( (char *)config->dev_eui, (char *)config->app_eui,
                               (char *)config->app_key, DIFFERENT_FREQ_MODE );
  if( err != ESP_OK )
  {
    ESP_LOGE( _TAG, "Failed to configure OTAA credentials" );
    return err;
  }

  // Configure TTN-specific network parameters
  err = _configure_ttn_network_parameters( config );
  if( err != ESP_OK )
  {
    ESP_LOGE( _TAG, "Failed to configure TTN network parameters" );
    return err;
  }

  // Save configuration to module
  lorawan_response_t response = { 0 };
  err = _unit_lorawan_send_at_command( "CSAVE", &response,
                                       UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );
  if( err == ESP_OK && response.success )
  {
    ESP_LOGI( _TAG, "✓ TTN configuration saved to module" );
  }
  else
  {
    ESP_LOGW( _TAG, "⚠ Failed to save TTN configuration" );
  }
  _unit_lorawan_cleanup_response( &response );

  // Initiate network join
  ESP_LOGI( _TAG, "Initiating TTN network join..." );
  err = unit_lorawan_join();
  if( err != ESP_OK )
  {
    ESP_LOGE( _TAG, "Failed to initiate TTN network join" );
    return err;
  }

  // If callback provided, create async join monitoring task
  if( join_callback )
  {
    typedef struct
    {
      unit_lorawan_ttn_join_callback_t callback;
      void *user_data;
      uint16_t timeout_sec;
    } join_monitor_params_t;

    join_monitor_params_t *params = malloc( sizeof( join_monitor_params_t ) );
    if( params )
    {
      params->callback = join_callback;
      params->user_data = user_data;
      params->timeout_sec = config->join_timeout_sec;

      // Create monitoring task
      xTaskCreate( ttn_join_monitor_task, "ttn_join_monitor", 4096, params, 3,
                   NULL );
    }
    else
    {
      ESP_LOGW( _TAG, "Failed to allocate memory for join monitoring task" );
    }
  }

  ESP_LOGI( _TAG, "✓ TTN US915 configuration completed successfully" );
  ESP_LOGI( _TAG, "  Join process initiated - %s",
            join_callback ? "callback will notify of result"
                          : "use unit_lorawan_connected() to check status" );

  return ESP_OK;
}

esp_err_t unit_lorawan_config_otaa_from_kconfig( void )
{
#ifdef CONFIG_LORAWAN_OTAA
  ESP_LOGI( _TAG, "Configuring OTAA from Kconfig values" );

  // Validate Kconfig values
  if( strlen( CONFIG_LORAWAN_DEVICE_EUI ) != UNIT_LORAWAN_EUI_LENGTH )
  {
    ESP_LOGE( _TAG,
              "Invalid CONFIG_LORAWAN_DEVICE_EUI length: %zu (expected %d)",
              strlen( CONFIG_LORAWAN_DEVICE_EUI ), UNIT_LORAWAN_EUI_LENGTH );
    return ESP_ERR_INVALID_ARG;
  }

  if( strlen( CONFIG_LORAWAN_APP_EUI ) != UNIT_LORAWAN_EUI_LENGTH )
  {
    ESP_LOGE( _TAG, "Invalid CONFIG_LORAWAN_APP_EUI length: %zu (expected %d)",
              strlen( CONFIG_LORAWAN_APP_EUI ), UNIT_LORAWAN_EUI_LENGTH );
    return ESP_ERR_INVALID_ARG;
  }

  if( strlen( CONFIG_LORAWAN_APP_KEY ) != UNIT_LORAWAN_APP_KEY_LENGTH )
  {
    ESP_LOGE( _TAG, "Invalid CONFIG_LORAWAN_APP_KEY length: %zu (expected %d)",
              strlen( CONFIG_LORAWAN_APP_KEY ), UNIT_LORAWAN_APP_KEY_LENGTH );
    return ESP_ERR_INVALID_ARG;
  }

  // Configure OTAA with Kconfig values - fix conditional compilation
#ifdef CONFIG_LORAWAN_ULDL_MODE
  unit_lorwan_uldlmode mode =
      CONFIG_LORAWAN_ULDL_MODE ? SAME_FREQ_MODE : DIFFERENT_FREQ_MODE;
#else
  unit_lorwan_uldlmode mode =
      DIFFERENT_FREQ_MODE; // Default to different frequency (recommended for
                           // TTN)
#endif

  esp_err_t err = unit_lorawan_configOTTA( CONFIG_LORAWAN_DEVICE_EUI,
                                           CONFIG_LORAWAN_APP_EUI,
                                           CONFIG_LORAWAN_APP_KEY, mode );

  if( err == ESP_OK )
  {
    ESP_LOGI( _TAG, "✓ OTAA configured from Kconfig:" );
    ESP_LOGI( _TAG, "  DevEUI: %s", CONFIG_LORAWAN_DEVICE_EUI );
    ESP_LOGI( _TAG, "  AppEUI: %s", CONFIG_LORAWAN_APP_EUI );
#ifdef CONFIG_LORAWAN_ULDL_MODE
    ESP_LOGI( _TAG, "  Mode: %s",
              CONFIG_LORAWAN_ULDL_MODE ? "Same frequency"
                                       : "Different frequency" );
#else
    ESP_LOGI( _TAG, "  Mode: Different frequency (default)" );
#endif
  }

  return err;
#else
  ESP_LOGE( _TAG, "CONFIG_LORAWAN_OTAA not enabled in Kconfig" );
  return ESP_ERR_INVALID_ARG;
#endif
}

esp_err_t unit_lorawan_config_abp_from_kconfig( void )
{
#ifdef CONFIG_LORAWAN_ABP
  ESP_LOGW( _TAG, "ABP configuration from Kconfig not fully implemented yet" );
  ESP_LOGW( _TAG, "ABP support is experimental - use OTAA for production" );

  // TODO: Implement ABP configuration
  ESP_LOGI( _TAG, "ABP Kconfig values:" );
  ESP_LOGI( _TAG, "  DevAddr: 0x%08X", CONFIG_LORAWAN_DEV_ADDR );
  ESP_LOGI( _TAG, "  AppSKey: %s", CONFIG_LORAWAN_APP_SKEY );
  ESP_LOGI( _TAG, "  NwkSKey: %s", CONFIG_LORAWAN_NWK_SKEY );

  return ESP_ERR_NOT_SUPPORTED;
#else
  ESP_LOGE( _TAG, "CONFIG_LORAWAN_ABP not enabled in Kconfig" );
  return ESP_ERR_INVALID_ARG;
#endif
}

esp_err_t
unit_lorawan_init_with_config( unit_lorawan_ttn_join_callback_t join_callback,
                               void *user_data )
{
  ESP_LOGI( _TAG, "Initializing LoRaWAN with Kconfig configuration" );

  // Initialize the basic LoRaWAN driver
  esp_err_t err = unit_lorawan_init();
  if( err != ESP_OK )
  {
    ESP_LOGE( _TAG, "Failed to initialize LoRaWAN driver" );
    return err;
  }

#ifdef CONFIG_LORAWAN_REGION_US915
  ESP_LOGI( _TAG, "Configuring for US915 region" );

  // Create TTN configuration from Kconfig values
  unit_lorawan_ttn_config_t ttn_config = {
#ifdef CONFIG_LORAWAN_OTAA
      .dev_eui = CONFIG_LORAWAN_DEVICE_EUI,
      .app_eui = CONFIG_LORAWAN_APP_EUI,
      .app_key = CONFIG_LORAWAN_APP_KEY,
#else
      .dev_eui = "0000000000000000", // Placeholder for ABP
      .app_eui = "0000000000000000",
      .app_key = "00000000000000000000000000000000",
#endif
#ifdef CONFIG_LORAWAN_US915_SUB_BAND
      .sub_band = CONFIG_LORAWAN_US915_SUB_BAND,
#else
      .sub_band = UNIT_LORAWAN_TTN_US915_SUB_BAND_DEFAULT,
#endif
#ifdef CONFIG_LORAWAN_US915_DATA_RATE
      .data_rate = CONFIG_LORAWAN_US915_DATA_RATE,
#else
      .data_rate = UNIT_LORAWAN_TTN_US915_DATA_RATE_DEFAULT,
#endif
#ifdef CONFIG_LORAWAN_ADR_ENABLED
      .adr_enabled = CONFIG_LORAWAN_ADR_ENABLED,
#else
      .adr_enabled = UNIT_LORAWAN_TTN_US915_ADR_ENABLED,
#endif
      .rx2_frequency = UNIT_LORAWAN_TTN_US915_RX2_FREQUENCY,
      .rx2_data_rate = UNIT_LORAWAN_TTN_US915_RX2_DATA_RATE,
#ifdef CONFIG_LORAWAN_JOIN_TIMEOUT_SEC
      .join_timeout_sec = CONFIG_LORAWAN_JOIN_TIMEOUT_SEC
#else
      .join_timeout_sec = UNIT_LORAWAN_TTN_US915_JOIN_TIMEOUT_SEC
#endif
  };

  // Configure TTN US915 with Kconfig values
  err =
      unit_lorawan_configure_ttn_us915( &ttn_config, join_callback, user_data );
  if( err != ESP_OK )
  {
    ESP_LOGE( _TAG, "Failed to configure TTN US915 with Kconfig values" );
    return err;
  }

  // Apply additional Kconfig settings
  ESP_LOGI( _TAG, "Applying additional Kconfig settings..." );

  // Set TX power from Kconfig
#ifdef CONFIG_LORAWAN_TX_POWER_INDEX
  err = unit_lorawan_set_tx_power( CONFIG_LORAWAN_TX_POWER_INDEX );
  if( err != ESP_OK )
  {
    ESP_LOGW( _TAG, "Failed to set TX power from Kconfig, using default" );
  }
#endif

  // Set retries from Kconfig
#ifdef CONFIG_LORAWAN_CONFIRMED_RETRIES
  err = unit_lorawan_set_retries( 1, CONFIG_LORAWAN_CONFIRMED_RETRIES );
  if( err != ESP_OK )
  {
    ESP_LOGW( _TAG, "Failed to set retries from Kconfig, using default" );
  }
#endif

  ESP_LOGI( _TAG, "✓ LoRaWAN configured from Kconfig:" );
  ESP_LOGI( _TAG, "  Region: US915" );
#ifdef CONFIG_LORAWAN_US915_SUB_BAND
  ESP_LOGI( _TAG, "  Sub-band: %d (channels %d-%d)",
            CONFIG_LORAWAN_US915_SUB_BAND,
            ( CONFIG_LORAWAN_US915_SUB_BAND - 1 ) * 8,
            ( CONFIG_LORAWAN_US915_SUB_BAND - 1 ) * 8 + 7 );
#endif
#ifdef CONFIG_LORAWAN_US915_DATA_RATE
  ESP_LOGI( _TAG, "  Data Rate: DR%d", CONFIG_LORAWAN_US915_DATA_RATE );
#endif
#ifdef CONFIG_LORAWAN_ADR_ENABLED
  ESP_LOGI( _TAG, "  ADR: %s",
            CONFIG_LORAWAN_ADR_ENABLED ? "Enabled" : "Disabled" );
#endif
#ifdef CONFIG_LORAWAN_TX_POWER_INDEX
  ESP_LOGI( _TAG, "  TX Power Index: %d", CONFIG_LORAWAN_TX_POWER_INDEX );
#endif
#ifdef CONFIG_LORAWAN_CONFIRMED_RETRIES
  ESP_LOGI( _TAG, "  Confirmed Retries: %d", CONFIG_LORAWAN_CONFIRMED_RETRIES );
#endif

#ifdef CONFIG_LORAWAN_OTAA
  ESP_LOGI( _TAG, "  Activation: OTAA" );
#else
  ESP_LOGI( _TAG, "  Activation: ABP (experimental)" );
#endif

#else
  ESP_LOGE( _TAG, "Unsupported region in Kconfig" );
  return ESP_ERR_NOT_SUPPORTED;
#endif

  return ESP_OK;
}

esp_err_t unit_lorawan_get_data_rate_info( uint8_t *current_data_rate,
                                           size_t *max_payload_size )
{
  if( !current_data_rate || !max_payload_size )
  {
    ESP_LOGE( _TAG, "Data rate and payload size parameters cannot be NULL" );
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGD( _TAG, "Querying current data rate and payload size" );

  lorawan_response_t response = { 0 };
  esp_err_t err = _unit_lorawan_send_at_command(
      "CDATARATE?", &response, UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );

  if( err == ESP_OK && response.success && response.response_data )
  {
    char *cdatarate_pos = strstr( response.response_data, "+CDATARATE:" );
    if( cdatarate_pos )
    {
      int dr_value = 0;
      if( sscanf( cdatarate_pos, "+CDATARATE:%d", &dr_value ) == 1 )
      {
        *current_data_rate = (uint8_t)dr_value;

        // Get max payload size for US915 data rates
        if( dr_value >= 0 && dr_value <= 4 )
        {
          *max_payload_size = us915_max_payload_sizes[ dr_value ];
          ESP_LOGI( _TAG, "Current data rate: DR%d, max payload: %zu bytes",
                    dr_value, *max_payload_size );
        }
        else
        {
          ESP_LOGW( _TAG,
                    "Unknown data rate %d, using conservative payload limit",
                    dr_value );
          *max_payload_size = UNIT_LORAWAN_US915_MAX_PAYLOAD_DR0;
        }
      }
      else
      {
        ESP_LOGE( _TAG, "Failed to parse data rate from CDATARATE response" );
        err = ESP_FAIL;
      }
    }
    else
    {
      ESP_LOGE( _TAG, "CDATARATE data not found in response: %s",
                response.response_data );
      err = ESP_FAIL;
    }
  }
  else
  {
    ESP_LOGW( _TAG, "Failed to query current data rate with CDATARATE" );
    _unit_lorawan_cleanup_response( &response );

    // Try alternative query
    err = _unit_lorawan_send_at_command( "CSTATUS?", &response,
                                         UNIT_LORAWAN_RESPONSE_TIMEOUT_MS );
    if( err == ESP_OK && response.success )
    {
      ESP_LOGW( _TAG, "Using default data rate DR2 due to query failure" );
      *current_data_rate = UNIT_LORAWAN_TTN_US915_DATA_RATE_DEFAULT;
      *max_payload_size =
          us915_max_payload_sizes[ UNIT_LORAWAN_TTN_US915_DATA_RATE_DEFAULT ];
    }
    else
    {
      ESP_LOGE( _TAG, "All data rate query attempts failed" );
    }
  }

  _unit_lorawan_cleanup_response( &response );
  return err;
}
