# M5Stack LoRaWAN Unit ESP-IDF Component

This is a comprehensive component library for use with the M5Stack LoRaWAN unit (ASR6501 based) on the Core2 for AWS IoT Kit connected to port C. The library provides a robust, feature-rich interface for LoRaWAN communication with extensive support for The Things Network (TTN) and other LoRaWAN networks.

## Features

- **Complete TTN US915 Integration**: One-function setup for The Things Network with optimal configuration
- **ASR6501 Full Feature Support**: Comprehensive AT command implementation based on official datasheet v4.0
- **Intelligent Payload Management**: Automatic validation and optimization against LoRaWAN data rate limits
- **Advanced Network Management**: Channel management, link checking, and diagnostics
- **Resilient Communication**: Automatic retry mechanisms, error handling, and recovery
- **Power Management**: TX power control, low power modes, and battery monitoring
- **Real-time Diagnostics**: Channel RSSI scanning, link quality assessment
- **Flexible Configuration**: Support for all US915 sub-bands, data rates, and network settings
- **Asynchronous Operations**: Non-blocking join with callback support and background monitoring
- **Memory Safety**: Proper allocation, cleanup, and resource management
- **Human-Readable Logging**: Detailed status messages with troubleshooting guidance

## Hardware Requirements

- M5Stack Core2 for AWS IoT Kit
- M5Stack LoRaWAN Unit (915MHz for US915 band, based on ASR6501)
- Connection to expansion port C (UART TX: GPIO14, RX: GPIO13)
- Appropriate antenna for your frequency band (915MHz for US915)
- Stable power supply (LoRaWAN transmission requires adequate power)

## Configuration Constants

The library provides comprehensive predefined constants for easy configuration:

### TTN US915 Constants
```c
#define UNIT_LORAWAN_TTN_US915_RX2_FREQUENCY        923300000   // TTN RX2 frequency (Hz) - Note: Auto-configured by stack
#define UNIT_LORAWAN_TTN_US915_RX2_DATA_RATE        8           // TTN RX2 data rate - Note: Auto-configured by stack
#define UNIT_LORAWAN_TTN_US915_SUB_BAND_DEFAULT     2           // TTN sub-band (channels 8-15)
#define UNIT_LORAWAN_TTN_US915_DATA_RATE_DEFAULT    2           // Recommended data rate (DR2)
#define UNIT_LORAWAN_TTN_US915_ADR_ENABLED          true        // Enable ADR
#define UNIT_LORAWAN_TTN_US915_JOIN_TIMEOUT_SEC     60          // Join timeout
#define UNIT_LORAWAN_TTN_V3_APP_EUI                 "0000000000000000"  // TTN v3 AppEUI
```

### US915 Data Rate and Payload Constants (Per ASR6501 Datasheet)
```c
#define UNIT_LORAWAN_US915_DATA_RATE_MIN            0           // DR0 (longest range)
#define UNIT_LORAWAN_US915_DATA_RATE_MAX            4           // DR4 (shortest range)
#define UNIT_LORAWAN_US915_MAX_PAYLOAD_DR0          11          // 11 bytes max (SF10, 125kHz)
#define UNIT_LORAWAN_US915_MAX_PAYLOAD_DR1          53          // 53 bytes max (SF9, 125kHz)
#define UNIT_LORAWAN_US915_MAX_PAYLOAD_DR2          125         // 125 bytes max (SF8, 125kHz)
#define UNIT_LORAWAN_US915_MAX_PAYLOAD_DR3          242         // 242 bytes max (SF7, 125kHz)
#define UNIT_LORAWAN_US915_MAX_PAYLOAD_DR4          242         // 242 bytes max (SF8, 500kHz)
```

### Safe Message Size Constant
```c
#define UNIT_LORAWAN_MAX_MESSAGE_SIZE               11          // Safe for all data rates
```

## API Reference

### Key Functions

#### `unit_lorawan_configure_ttn_us915()`

Complete one-step configuration for The Things Network (TTN) US915 with optimal settings based on ASR6501 capabilities.

```c
esp_err_t unit_lorawan_configure_ttn_us915(const unit_lorawan_ttn_config_t *config,
                                          unit_lorawan_ttn_join_callback_t join_callback,
                                          void *user_data);
```

**Configuration Structure:**
```c
typedef struct {
    const char *dev_eui;          // Device EUI (16 hex characters)
    const char *app_eui;          // Application EUI (16 hex characters)
    const char *app_key;          // Application Key (32 hex characters)
    uint8_t sub_band;             // US915 sub-band (1-8)
    uint8_t data_rate;            // Initial data rate (0-4)
    bool adr_enabled;             // Enable Adaptive Data Rate
    uint32_t rx2_frequency;       // RX2 frequency in Hz (for config completeness)
    uint8_t rx2_data_rate;        // RX2 data rate (for config completeness)
    uint16_t join_timeout_sec;    // Join timeout in seconds
} unit_lorawan_ttn_config_t;
```

**Note:** RX2 parameters are automatically configured by the ASR6501 stack per US915 regional parameters and cannot be manually set.

#### `unit_lorawan_get_data_rate_info()`

Gets current data rate and maximum payload size for optimal message sizing.

```c
esp_err_t unit_lorawan_get_data_rate_info(uint8_t *current_data_rate, size_t *max_payload_size);
```

#### `unit_lorawan_send()`

Sends an uplink message with automatic payload validation using the correct DTRX format per ASR6501 datasheet.

```c
esp_err_t unit_lorawan_send(char *message, size_t length);
```

## Complete TTN US915 Example

Production-ready example with proper payload management and error handling:

```c
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "core2foraws.h"
#include "unit_lorawan.h"

static const char *TAG = "TTN_EXAMPLE";

// TTN application credentials - replace with your actual values from TTN console
#define DEV_EUI     "1234567890ABCDEF"  // Replace with your DevEUI
#define APP_KEY     "ABCDEFGHIJKLMNO1234567890"  // Replace with your AppKey

static bool network_joined = false;

void ttn_join_callback( bool joined , uint8_t error_code , void * user_data )
{
    if( joined )
    {
        ESP_LOGI( TAG , "✓ Successfully joined TTN network!" ) ;
        network_joined = true ;
    }
    else
    {
        ESP_LOGE( TAG , "✗ Failed to join TTN network, error code: %d" , error_code ) ;
        if( error_code == 1 )
        {
            ESP_LOGE( TAG , "Join timeout - check credentials and gateway coverage" ) ;
        }
    }
}

void send_sensor_data( int message_count , float temperature , float humidity )
{
    uint8_t current_dr ;
    size_t max_payload ;
    esp_err_t err = unit_lorawan_get_data_rate_info( &current_dr , &max_payload ) ;
    if( err != ESP_OK )
    {
        ESP_LOGE( TAG , "Failed to get data rate info, using conservative estimate" ) ;
        current_dr = UNIT_LORAWAN_TTN_US915_DATA_RATE_DEFAULT ;
        max_payload = UNIT_LORAWAN_US915_MAX_PAYLOAD_DR2 ;
    }
    uint8_t tx_power = 0 ;
    unit_lorawan_get_tx_power( &tx_power ) ;
    char message [ UNIT_LORAWAN_US915_MAX_PAYLOAD_DR3 + 1 ] ;
    int msg_len ;
    if( max_payload >= 50 )
    {
        msg_len = snprintf( message , sizeof( message ) ,
                "MSG_%03d,T:%.1f,H:%.1f,DR:%d,PWR:%d" ,
                message_count , temperature , humidity , current_dr , tx_power ) ;
    }
    else if( max_payload >= 25 )
    {
        msg_len = snprintf( message , sizeof( message ) ,
                "M%03d,T:%.1f,H:%.1f" ,
                message_count , temperature , humidity ) ;
    }
    else
    {
        msg_len = snprintf( message , sizeof( message ) ,
                "T%.0fH%.0f" , temperature , humidity ) ;
    }
    if( msg_len > 0 && ( size_t ) msg_len <= max_payload )
    {
        err = unit_lorawan_send( message , msg_len ) ;
        if( err == ESP_OK )
        {
            ESP_LOGI( TAG , "✓ Sent (%d/%zu bytes) on DR%d: %s" ,
                    msg_len , max_payload , current_dr , message ) ;
        }
        else
        {
            ESP_LOGE( TAG , "✗ Failed to send message: %s" , esp_err_to_name( err ) ) ;
        }
    }
    else
    {
        ESP_LOGW( TAG , "⚠ Message too large (%d bytes) for DR%d (max: %zu bytes)" ,
                 msg_len , current_dr , max_payload ) ;
        char min_msg [ 12 ] ;
        int min_len = snprintf( min_msg , sizeof( min_msg ) , "T%.0fH%.0f" , temperature , humidity ) ;
        if( min_len > 0 && min_len <= 11 )
        {
            err = unit_lorawan_send( min_msg , min_len ) ;
            if( err == ESP_OK )
            {
                ESP_LOGI( TAG , "✓ Sent minimal message (%d bytes): %s" , min_len , min_msg ) ;
            }
        }
    }
}

void ttn_communication_task( void * pvParameters )
{
    ESP_LOGI( TAG , "Starting TTN LoRaWAN communication" ) ;
    esp_err_t err = unit_lorawan_init() ;
    if( err != ESP_OK )
    {
        ESP_LOGE( TAG , "Failed to initialize LoRaWAN driver" ) ;
        vTaskDelete( NULL ) ;
        return ;
    }
    unit_lorawan_ttn_config_t ttn_config = { 
        .dev_eui = DEV_EUI ,
        .app_eui = UNIT_LORAWAN_TTN_V3_APP_EUI ,
        .app_key = APP_KEY ,
        .sub_band = UNIT_LORAWAN_TTN_US915_SUB_BAND_DEFAULT ,
        .data_rate = UNIT_LORAWAN_TTN_US915_DATA_RATE_DEFAULT ,
        .adr_enabled = UNIT_LORAWAN_TTN_US915_ADR_ENABLED ,
        .rx2_frequency = UNIT_LORAWAN_TTN_US915_RX2_FREQUENCY ,
        .rx2_data_rate = UNIT_LORAWAN_TTN_US915_RX2_DATA_RATE ,
        .join_timeout_sec = UNIT_LORAWAN_TTN_US915_JOIN_TIMEOUT_SEC 
    } ;
    err = unit_lorawan_configure_ttn_us915( &ttn_config , ttn_join_callback , NULL ) ;
    if( err != ESP_OK )
    {
        ESP_LOGE( TAG , "Failed to configure TTN: %d" , err ) ;
        vTaskDelete( NULL ) ;
        return ;
    }
    ESP_LOGI( TAG , "Waiting for network join..." ) ;
    while( ! network_joined )
    {
        vTaskDelay( pdMS_TO_TICKS( 1000 ) ) ;
    }
    unit_lorawan_save_config() ;
    unit_lorawan_set_retries( 1 , 3 ) ;
    unit_lorawan_set_tx_power( 2 ) ;
    ESP_LOGI( TAG , "Network joined successfully, starting data transmission" ) ;
    int message_count = 0 ;
    while( 1 )
    {
        float temperature = 25.5 + ( float )( esp_random() % 100 ) / 10.0 ;
        float humidity = 60.0 + ( float )( esp_random() % 400 ) / 10.0 ;
        send_sensor_data( message_count ++ , temperature , humidity ) ;
        if( message_count % 10 == 0 )
        {
            ESP_LOGI( TAG , "Performing link check..." ) ;
            unit_lorawan_link_check( 1 ) ;
        }
        ESP_LOGI( TAG , "Waiting 10 minutes before next message..." ) ;
        vTaskDelay( pdMS_TO_TICKS( 600000 ) ) ;
    }
}

void app_main( void )
{
    core2foraws_init() ;
    core2foraws_expports_i2c_begin() ;
    ESP_LOGI( TAG , "TTN US915 LoRaWAN Example Application" ) ;
    ESP_LOGI( TAG , "Configuration:" ) ;
    ESP_LOGI( TAG , "  Sub-band: %d (channels %d-%d)" ,
             UNIT_LORAWAN_TTN_US915_SUB_BAND_DEFAULT ,
             ( UNIT_LORAWAN_TTN_US915_SUB_BAND_DEFAULT - 1 ) * 8 ,
             ( UNIT_LORAWAN_TTN_US915_SUB_BAND_DEFAULT - 1 ) * 8 + 7 ) ;
    ESP_LOGI( TAG , "  Data Rate: DR%d (max %d bytes payload)" ,
             UNIT_LORAWAN_TTN_US915_DATA_RATE_DEFAULT ,
             UNIT_LORAWAN_US915_MAX_PAYLOAD_DR2 ) ;
    ESP_LOGI( TAG , "  ADR: %s" , UNIT_LORAWAN_TTN_US915_ADR_ENABLED ? "Enabled" : "Disabled" ) ;
    xTaskCreatePinnedToCore( 
        ttn_communication_task ,
        "ttn_lorawan" ,
        8192 ,
        NULL ,
        5 ,
        NULL ,
        1 
    ) ;
}
```

## ASR6501 Specific Notes

Based on the official ASR6501 datasheet v4.0:

### Supported Features
- **Data Rate Command**: `AT+CDATARATE=<value>` (0-5, where 0=SF12 to 5=SF7)
- **TX Power Command**: `AT+CTXP=<value>` (0-7, power index)
- **Channel RSSI**: `AT+CRSSI <FREQBANDIDX>?` (returns RSSI for all 8 channels in a frequency band)
- **Link Check**: `AT+CLINKCHECK=<value>` (0=disable, 1=once, 2=after every uplink)
- **Send Command**: `AT+DTRX=[confirm],[nbtrials],<Length>,<Payload>` (hex payload)

### Auto-Configured Features
- **RX2 Parameters**: Automatically handled by the LoRaWAN stack per regional parameters
- **Frequency Plans**: Configured via `CFREQBANDMASK` command with sub-band masks

## RSSI Diagnostics

The ASR6501 supports channel RSSI scanning for network diagnostics:

```c
// Scan RSSI for frequency band 1 (group 1A2)
int16_t rssi_values[ 8 ];
size_t channel_count;
esp_err_t err = unit_lorawan_get_channel_rssi( 1 , rssi_values , &channel_count ) ;
if( err == ESP_OK )
{
    for( size_t i = 0; i < channel_count; i++ )
    {
        ESP_LOGI( TAG , "Channel %zu: %d dBm" , i , rssi_values[ i ] ) ;
    }
}
```

**Note**: Individual RSSI queries are not supported by the ASR6501. Only frequency band channel scanning is available.

## Troubleshooting

### Module Not Detected
- **Check physical connections** to expansion port C
- **Verify 915MHz antenna** is properly connected
- **Try module reboot**: `unit_lorawan_reboot()`

### TTN Join Failures
- **Verify credentials** match TTN console exactly
- **Use correct AppEUI**: Set to `UNIT_LORAWAN_TTN_V3_APP_EUI` for TTN v3
- **Check gateway coverage** using TTN Mapper
- **Verify sub-band**: Use sub-band 2 for TTN US915

### Send Failures
- **Check payload size** against current data rate limits
- **Respect TTN Fair Use Policy** (30s airtime/day, 10 downlinks/day)
- **Use conservative message sizes** for reliability

## Performance Tips

1. **Use DR2 as default** for good range/payload balance
2. **Enable ADR** for automatic network optimization
3. **Validate payload sizes** against current data rate
4. **Implement fallback** to smaller messages for lower data rates
5. **Monitor link quality** with periodic link checks

## License

Licensed under the Apache License, Version 2.0. See LICENSE file for details.
