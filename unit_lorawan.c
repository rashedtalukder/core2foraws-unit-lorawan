/*!
 * @file unit_lorawan.c
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
 * @see [Unit LoRaWAN915](https://docs.m5stack.com/en/unit/lorawan915)
 * @version 1.0.0
 * @date 2026-09-26
 */

#include "unit_lorawan.h"

#include "core2foraws.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <limits.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined( __has_include )
#if __has_include( "sdkconfig.h" )
#include "sdkconfig.h"
#endif
#endif

#ifndef CONFIG_LORAWAN_DEVICE_EUI
#define CONFIG_LORAWAN_DEVICE_EUI ""
#endif
#ifndef CONFIG_LORAWAN_APP_EUI
#define CONFIG_LORAWAN_APP_EUI "0000000000000000"
#endif
#ifndef CONFIG_LORAWAN_APP_KEY
#define CONFIG_LORAWAN_APP_KEY ""
#endif
#ifndef CONFIG_LORAWAN_DEV_ADDR
#define CONFIG_LORAWAN_DEV_ADDR ""
#endif
#ifndef CONFIG_LORAWAN_APP_SKEY
#define CONFIG_LORAWAN_APP_SKEY ""
#endif
#ifndef CONFIG_LORAWAN_NWK_SKEY
#define CONFIG_LORAWAN_NWK_SKEY ""
#endif
#ifndef CONFIG_LORAWAN_US915_SUB_BAND
#define CONFIG_LORAWAN_US915_SUB_BAND 2
#endif
#ifndef CONFIG_LORAWAN_US915_DATA_RATE
#define CONFIG_LORAWAN_US915_DATA_RATE 3
#endif
#ifndef CONFIG_LORAWAN_TX_POWER_INDEX
#define CONFIG_LORAWAN_TX_POWER_INDEX 0
#endif
#ifndef CONFIG_LORAWAN_CONFIRMED_RETRIES
#define CONFIG_LORAWAN_CONFIRMED_RETRIES 3
#endif
#ifndef CONFIG_LORAWAN_APP_PORT
#define CONFIG_LORAWAN_APP_PORT 10
#endif

#define LW_LINE_MAX          600
#define LW_REPLY_MAX         1024
#define LW_WIRE_MAX          560
#define LW_EVENT_DEPTH       8
#define LW_READ_CHUNK        128
#define LW_TIMEOUT_MS        2000
#define LW_FLASH_TIMEOUT_MS  5000
#define LW_RSSI_TIMEOUT_MS   3000
#define LW_PROBE_TIMEOUT_MS  400
#define LW_LOCK_TIMEOUT_MS   120000
#define LW_SERVICE_PERIOD_MS 20
#define LW_UPLINK_GRACE_MS   300
#define LW_WAKE_SETTLE_MS    30
#define LW_JOIN_POLL_MS      5000
#define LW_BOOT_TIMEOUT_MS   6000
#define LW_BOOT_AFTER_TX_MS  30000
#define LW_SLEEP_TIMER_MS    10500

typedef enum
{
  LW_TXN_NONE = 0,
  LW_TXN_GENERIC,
  LW_TXN_UPLINK,
  LW_TXN_TEST, // may never print OK; any output counts as accepted
} lw_txn_kind_t;

typedef struct
{
  lw_txn_kind_t kind;
  bool done;
  esp_err_t result;
  bool any_line;
  bool truncated;
  bool sent;
  TickType_t sent_tick;
  unit_lorawan_tx_result_t tx;
  size_t reply_len;
  char reply[ LW_REPLY_MAX ];
} lw_txn_t;

static struct
{
  SemaphoreHandle_t lock;
  atomic_bool ready;
  atomic_bool task_run;
  atomic_bool task_exited;
  TaskHandle_t task;
  uint32_t baud;
  unit_lorawan_event_cb_t cb;
  void *cb_ctx;
  unit_lorawan_info_t info;

  char line[ LW_LINE_MAX ];
  size_t line_len;
  bool line_overflow;

  char wire[ LW_WIRE_MAX ];
  size_t wire_len;
  lw_txn_t txn;

  unit_lorawan_event_t events[ LW_EVENT_DEPTH ];
  uint8_t event_head;
  uint8_t event_count;

  atomic_int join_state;
  TickType_t join_start;
  TickType_t join_ticks;

  bool low_power;
  bool wake_next;
  bool sleeping;
  TickType_t sleep_until;
  bool test_mode;

  bool default_confirmed;
  uint8_t trials[ 2 ];
  bool trials_known[ 2 ];
  uint8_t port;
  uint8_t data_rate;
  bool adr;

  unit_lorawan_link_check_t link_check;
  bool link_check_valid;
  unit_lorawan_stats_t stats;
} s_lw;

static const char *_TAG = "UNIT_LORAWAN";
static const uint8_t _us915_max_payload[] = { 11, 53, 125, 242, 242 };
static const char _hex_digits[] = "0123456789ABCDEF";
static const uint32_t _baud_rates[] = { 115200, 57600, 38400, 19200, 9600 };

/* ---- Small helpers ------------------------------------------------------ */

static TickType_t _now( void ) { return xTaskGetTickCount(); }

static bool _elapsed( TickType_t since, TickType_t ticks )
{
  return (TickType_t)( _now() - since ) >= ticks;
}

static bool _starts( const char *text, const char *prefix )
{
  return strncmp( text, prefix, strlen( prefix ) ) == 0;
}

static int _hex_value( char c )
{
  if( c >= '0' && c <= '9' ) return c - '0';
  c = (char)( c | 0x20 );
  if( c >= 'a' && c <= 'f' ) return c - 'a' + 10;
  return -1;
}

static bool _is_hex( const char *value, size_t length )
{
  if( value == NULL || strlen( value ) != length ) return false;
  for( size_t i = 0; i < length; i++ )
    if( _hex_value( value[ i ] ) < 0 ) return false;
  return true;
}

static bool _parse_ulong( const char *text, int base, unsigned long *out )
{
  char *end = NULL;
  if( text == NULL || *text == '\0' || *text == '-' ) return false;
  unsigned long value = strtoul( text, &end, base );
  if( end == text || ( *end != '\0' && *end != ',' && *end != '\n' ) )
    return false;
  *out = value;
  return true;
}

static bool _us915_frequency( uint32_t hz )
{
  return hz >= UNIT_LORAWAN_US915_FREQ_MIN_HZ &&
         hz <= UNIT_LORAWAN_US915_FREQ_MAX_HZ;
}

static bool _baud_supported( uint32_t baud )
{
  for( size_t i = 0; i < sizeof( _baud_rates ) / sizeof( _baud_rates[ 0 ] ); i++ )
    if( _baud_rates[ i ] == baud ) return true;
  return false;
}

/* ---- Events ------------------------------------------------------------- */

static unit_lorawan_event_t *_event_slot( unit_lorawan_event_id_t id )
{
  if( s_lw.event_count == LW_EVENT_DEPTH )
  {
    s_lw.event_head = ( s_lw.event_head + 1 ) % LW_EVENT_DEPTH;
    s_lw.event_count--;
    s_lw.stats.events_dropped++;
  }
  unit_lorawan_event_t *slot =
      &s_lw.events[ ( s_lw.event_head + s_lw.event_count ) % LW_EVENT_DEPTH ];
  s_lw.event_count++;
  memset( slot, 0, sizeof( *slot ) );
  slot->id = id;
  return slot;
}

static void _set_joined( bool joined, bool timed_out )
{
  int previous = atomic_exchange( &s_lw.join_state,
                                  joined ? UNIT_LORAWAN_JOIN_JOINED
                                         : UNIT_LORAWAN_JOIN_FAILED );
  if( joined && previous == UNIT_LORAWAN_JOIN_JOINED ) return;
  unit_lorawan_event_t *event = _event_slot(
      joined ? UNIT_LORAWAN_EVENT_JOINED : UNIT_LORAWAN_EVENT_JOIN_FAILED );
  event->join_timed_out = timed_out;
}

/* ---- Line processing ---------------------------------------------------- */

// Folds full-width commas and spaces after separators seen in vendor examples.
static void _normalize( char *line )
{
  const char *read = line;
  char *write = line;
  while( *read == ' ' || *read == '\t' ) read++;
  while( *read )
  {
    if( (uint8_t)read[ 0 ] == 0xEF && (uint8_t)read[ 1 ] == 0xBC &&
        (uint8_t)read[ 2 ] == 0x8C )
    {
      *write++ = ',';
      read += 3;
    }
    else if( *read == '\t' )
    {
      read++;
      continue;
    }
    else
    {
      *write++ = *read++;
    }
    if( write[ -1 ] == ',' || write[ -1 ] == ':' )
      while( *read == ' ' ) read++;
  }
  while( write > line && write[ -1 ] == ' ' ) write--;
  *write = '\0';
}

static bool _is_error_line( const char *line )
{
  return strcmp( line, "ERROR" ) == 0 || _starts( line, "ERROR:" ) ||
         _starts( line, "+CME ERROR" );
}

static void _append_reply( const char *line )
{
  size_t length = strlen( line );
  if( s_lw.txn.reply_len + length + 2 > sizeof( s_lw.txn.reply ) )
  {
    s_lw.txn.truncated = true;
    return;
  }
  memcpy( s_lw.txn.reply + s_lw.txn.reply_len, line, length );
  s_lw.txn.reply_len += length;
  s_lw.txn.reply[ s_lw.txn.reply_len++ ] = '\n';
  s_lw.txn.reply[ s_lw.txn.reply_len ] = '\0';
}

static void _complete( esp_err_t result )
{
  s_lw.txn.done = true;
  s_lw.txn.result = result;
}

static void _on_downlink( char *fields )
{
  char *token[ 4 ] = { fields, NULL, NULL, NULL };
  for( int i = 1; i < 4; i++ )
  {
    char *comma = strchr( token[ i - 1 ], ',' );
    if( comma == NULL )
    {
      if( i < 3 ) goto malformed;
      token[ 3 ] = token[ 2 ] + strlen( token[ 2 ] );
      break;
    }
    *comma = '\0';
    token[ i ] = comma + 1;
  }

  unsigned long type = 0, port = 0, len_hex = 0, len_dec = 0;
  if( !_parse_ulong( token[ 0 ], 16, &type ) ||
      !_parse_ulong( token[ 1 ], 16, &port ) ||
      !_parse_ulong( token[ 2 ], 16, &len_hex ) || type > 0xFF || port > 0xFF )
    goto malformed;
  if( !_parse_ulong( token[ 2 ], 10, &len_dec ) ) len_dec = ULONG_MAX;

  // TYPE/PORT/LEN are one-byte fields printed as two hex digits; the DATA
  // length is authoritative and LEN is accepted in either radix.
  size_t hex_length = strlen( token[ 3 ] );
  size_t length = hex_length / 2;
  if( ( hex_length & 1U ) || length > UNIT_LORAWAN_MAX_DOWNLINK ||
      ( len_hex != length && len_dec != length ) )
    goto malformed;
  for( size_t i = 0; i < hex_length; i++ )
    if( _hex_value( token[ 3 ][ i ] ) < 0 ) goto malformed;

  if( s_lw.txn.kind == LW_TXN_UPLINK )
  {
    s_lw.txn.tx.downlink = true;
    if( type & UNIT_LORAWAN_DL_ACK ) s_lw.txn.tx.acked = true;
  }
  if( length == 0 ) return;

  unit_lorawan_event_t *event = _event_slot( UNIT_LORAWAN_EVENT_DOWNLINK );
  event->downlink.port = (uint8_t)port;
  event->downlink.flags = (uint8_t)( type & 0x0F );
  event->downlink.length = (uint8_t)length;
  for( size_t i = 0; i < length; i++ )
    event->downlink.data[ i ] =
        (uint8_t)( ( _hex_value( token[ 3 ][ 2 * i ] ) << 4 ) |
                   _hex_value( token[ 3 ][ 2 * i + 1 ] ) );
  s_lw.stats.downlinks++;
  return;

malformed:
  s_lw.stats.lines_dropped++;
  ESP_LOGW( _TAG, "Malformed OK+RECV indication dropped" );
}

static bool _on_link_check( const char *fields )
{
  long value[ 5 ];
  const char *cursor = fields;
  for( int i = 0; i < 5; i++ )
  {
    char *end = NULL;
    value[ i ] = strtol( cursor, &end, 10 );
    if( end == cursor || ( i < 4 && *end != ',' ) || ( i == 4 && *end ) )
      return false;
    cursor = end + 1;
  }
  s_lw.link_check.success = value[ 0 ] == 0;
  s_lw.link_check.demod_margin_db = (uint8_t)value[ 1 ];
  s_lw.link_check.gateways = (uint8_t)value[ 2 ];
  s_lw.link_check.rssi_dbm = (int16_t)value[ 3 ];
  s_lw.link_check.snr_db = (int8_t)value[ 4 ];
  s_lw.link_check_valid = true;
  _event_slot( UNIT_LORAWAN_EVENT_LINK_CHECK )->link_check = s_lw.link_check;
  return true;
}

static void _on_uplink_line( const char *line )
{
  unit_lorawan_tx_result_t *tx = &s_lw.txn.tx;
  unsigned long value = 0;
  if( _starts( line, "OK+SEND:" ) ) return;
  if( _starts( line, "OK+SENT:" ) )
  {
    if( _parse_ulong( line + 8, 16, &value ) ) tx->transmissions = (uint8_t)value;
    s_lw.txn.sent = true;
    s_lw.txn.sent_tick = _now();
    return;
  }
  if( _starts( line, "ERR+SEND:" ) )
  {
    if( !_parse_ulong( line + 9, 16, &value ) ) value = ULONG_MAX;
    switch( value )
    {
    case 0:
      tx->status = UNIT_LORAWAN_TX_NOT_JOINED;
      _complete( ESP_ERR_INVALID_STATE );
      break;
    case 1:
      tx->status = UNIT_LORAWAN_TX_BUSY;
      _complete( ESP_ERR_NOT_FINISHED );
      break;
    case 2:
      tx->status = UNIT_LORAWAN_TX_TOO_LONG;
      _complete( ESP_ERR_INVALID_SIZE );
      break;
    default:
      tx->status = UNIT_LORAWAN_TX_REJECTED;
      _complete( ESP_FAIL );
      break;
    }
    return;
  }
  if( _starts( line, "ERR+SENT:" ) )
  {
    if( _parse_ulong( line + 9, 16, &value ) ) tx->transmissions = (uint8_t)value;
    tx->status = UNIT_LORAWAN_TX_NO_ACK;
    _complete( ESP_ERR_TIMEOUT );
    return;
  }
  if( _is_error_line( line ) )
  {
    _append_reply( line );
    tx->status = UNIT_LORAWAN_TX_REJECTED;
    _complete( ESP_FAIL );
  }
}

static void _process_line( char *line )
{
  _normalize( line );
  if( line[ 0 ] == '\0' ) return;
  if( line[ 0 ] == 'A' && line[ 1 ] == 'T' ) return; // command echo

  if( _starts( line, "+CJOIN:" ) )
  {
    if( strcmp( line + 7, "OK" ) == 0 )
    {
      _set_joined( true, false );
      return;
    }
    if( strcmp( line + 7, "FAIL" ) == 0 )
    {
      _set_joined( false, false );
      return;
    }
  }
  if( _starts( line, "OK+RECV:" ) )
  {
    _on_downlink( line + 8 );
    return;
  }
  if( _starts( line, "+CLINKCHECK:" ) && _on_link_check( line + 12 ) ) return;

  switch( s_lw.txn.kind )
  {
  case LW_TXN_NONE:
    return;
  case LW_TXN_UPLINK:
    _on_uplink_line( line );
    return;
  case LW_TXN_GENERIC:
  case LW_TXN_TEST:
    s_lw.txn.any_line = true;
    if( strcmp( line, "OK" ) == 0 )
    {
      _complete( ESP_OK );
    }
    else if( _is_error_line( line ) )
    {
      _append_reply( line );
      _complete( ESP_FAIL );
    }
    else
    {
      _append_reply( line );
    }
    return;
  }
}

static void _feed( const uint8_t *data, size_t length )
{
  for( size_t i = 0; i < length; i++ )
  {
    char c = (char)data[ i ];
    if( c == '\r' || c == '\n' )
    {
      if( s_lw.line_overflow )
      {
        s_lw.line_overflow = false;
        s_lw.stats.lines_dropped++;
      }
      else if( s_lw.line_len > 0 )
      {
        s_lw.line[ s_lw.line_len ] = '\0';
        _process_line( s_lw.line );
      }
      s_lw.line_len = 0;
    }
    else if( c != '\0' && !s_lw.line_overflow )
    {
      if( s_lw.line_len >= sizeof( s_lw.line ) - 1 )
      {
        s_lw.line_overflow = true;
        s_lw.line_len = 0;
      }
      else
      {
        s_lw.line[ s_lw.line_len++ ] = c;
      }
    }
  }
}

static esp_err_t _pump( void )
{
  uint8_t chunk[ LW_READ_CHUNK ];
  // Bounded so a babbling modem cannot starve the caller.
  for( int i = 0; i < 64; i++ )
  {
    size_t received = 0;
    esp_err_t err =
        core2foraws_expports_uart_read( chunk, sizeof( chunk ), &received );
    if( err != ESP_OK ) return err;
    if( received == 0 ) return ESP_OK;
    _feed( chunk, received );
  }
  return ESP_OK;
}

/* ---- Transactions (lock held) -------------------------------------------- */

__attribute__( ( format( printf, 1, 0 ) ) ) static esp_err_t
_vformat( const char *fmt, va_list args )
{
  memcpy( s_lw.wire, "AT+", 3 );
  int length = vsnprintf( s_lw.wire + 3, sizeof( s_lw.wire ) - 4, fmt, args );
  if( length < 0 || (size_t)length >= sizeof( s_lw.wire ) - 4 )
    return ESP_ERR_INVALID_SIZE;
  s_lw.wire[ 3 + length ] = '\r';
  s_lw.wire[ 4 + length ] = '\0';
  s_lw.wire_len = (size_t)length + 4;
  return ESP_OK;
}

__attribute__( ( format( printf, 1, 2 ) ) ) static esp_err_t
_format( const char *fmt, ... )
{
  va_list args;
  va_start( args, fmt );
  esp_err_t err = _vformat( fmt, args );
  va_end( args );
  return err;
}

static esp_err_t _write( const char *data, size_t length )
{
  size_t written = 0;
  esp_err_t err = core2foraws_expports_uart_write( data, length, &written );
  if( err == ESP_OK && written != length ) err = ESP_ERR_INVALID_SIZE;
  return err;
}

static void _prepare_modem( void )
{
  if( s_lw.sleeping )
  {
    TickType_t remaining = s_lw.sleep_until - _now();
    if( (int32_t)remaining > 0 ) vTaskDelay( remaining );
    s_lw.sleeping = false;
  }
  if( s_lw.low_power || s_lw.wake_next )
  {
    static const char wake[] = { 0, 0, 0, 0, '\r', '\n' };
    if( _write( wake, sizeof( wake ) ) == ESP_OK )
    {
      vTaskDelay( pdMS_TO_TICKS( LW_WAKE_SETTLE_MS ) );
      _pump();
    }
    s_lw.wake_next = false;
  }
}

static esp_err_t _transact_once( lw_txn_kind_t kind, uint32_t timeout_ms )
{
  esp_err_t err = _pump();
  if( err != ESP_OK ) return err;
  _prepare_modem();

  lw_txn_t *txn = &s_lw.txn;
  txn->done = false;
  txn->result = ESP_FAIL;
  txn->any_line = false;
  txn->truncated = false;
  txn->sent = false;
  memset( &txn->tx, 0, sizeof( txn->tx ) );
  txn->tx.status = UNIT_LORAWAN_TX_TIMEOUT;
  txn->reply_len = 0;
  txn->reply[ 0 ] = '\0';
  txn->kind = kind;

  err = _write( s_lw.wire, s_lw.wire_len );
  if( err != ESP_OK )
  {
    txn->kind = LW_TXN_NONE;
    s_lw.stats.command_errors++;
    return err;
  }
  s_lw.stats.commands++;

  const TickType_t start = _now();
  const TickType_t limit = pdMS_TO_TICKS( timeout_ms );
  const TickType_t grace = pdMS_TO_TICKS( LW_UPLINK_GRACE_MS );
  for( ;; )
  {
    err = _pump();
    if( err != ESP_OK ) break;
    if( txn->done )
    {
      err = txn->result;
      break;
    }
    if( kind == LW_TXN_UPLINK && txn->sent &&
        ( txn->tx.downlink || _elapsed( txn->sent_tick, grace ) ) )
    {
      txn->tx.status = UNIT_LORAWAN_TX_OK;
      err = ESP_OK;
      break;
    }
    if( _elapsed( start, limit ) )
    {
      err = ( kind == LW_TXN_TEST && txn->any_line ) ? ESP_OK : ESP_ERR_TIMEOUT;
      break;
    }
    vTaskDelay( 1 );
  }
  txn->kind = LW_TXN_NONE;

  if( err == ESP_ERR_TIMEOUT ) s_lw.stats.command_timeouts++;
  else if( err != ESP_OK ) s_lw.stats.command_errors++;
  return err;
}

static esp_err_t _transact( lw_txn_kind_t kind, uint32_t timeout_ms,
                            int attempts )
{
  if( s_lw.test_mode ) return ESP_ERR_INVALID_STATE;
  esp_err_t err = ESP_ERR_TIMEOUT;
  for( int i = 0; i < attempts; i++ )
  {
    err = _transact_once( kind, timeout_ms );
    if( err != ESP_ERR_TIMEOUT ) break;
  }
  return err;
}

__attribute__( ( format( printf, 3, 4 ) ) ) static esp_err_t
_run( uint32_t timeout_ms, int attempts, const char *fmt, ... )
{
  va_list args;
  va_start( args, fmt );
  esp_err_t err = _vformat( fmt, args );
  va_end( args );
  if( err != ESP_OK ) return err;
  return _transact( LW_TXN_GENERIC, timeout_ms, attempts );
}

// Value after "+NAME:" or "+NAME=" in the current reply.
static const char *_field( const char *name )
{
  size_t length = strlen( name );
  const char *line = s_lw.txn.reply;
  while( *line )
  {
    if( line[ 0 ] == '+' && strncmp( line + 1, name, length ) == 0 &&
        ( line[ 1 + length ] == ':' || line[ 1 + length ] == '=' ) )
      return line + 2 + length;
    const char *next = strchr( line, '\n' );
    if( next == NULL ) break;
    line = next + 1;
  }
  return NULL;
}

static size_t _field_ints( const char *name, long *values, size_t max )
{
  const char *cursor = _field( name );
  size_t count = 0;
  while( cursor && count < max )
  {
    char *end = NULL;
    long value = strtol( cursor, &end, 10 );
    if( end == cursor ) break;
    values[ count++ ] = value;
    if( *end != ',' ) break;
    cursor = end + 1;
  }
  return count;
}

static bool _field_copy( const char *name, char *out, size_t capacity )
{
  const char *value = _field( name );
  if( value == NULL || capacity == 0 ) return false;
  if( *value == '"' ) value++;
  size_t length = strcspn( value, "\n" );
  if( length && value[ length - 1 ] == '"' ) length--;
  if( length >= capacity ) length = capacity - 1;
  memcpy( out, value, length );
  out[ length ] = '\0';
  return true;
}

static esp_err_t _query_ints( const char *name, long *values, size_t count )
{
  esp_err_t err = _run( LW_TIMEOUT_MS, 2, "%s?", name );
  if( err == ESP_OK && _field_ints( name, values, count ) < count )
    err = ESP_ERR_INVALID_RESPONSE;
  return err;
}

static esp_err_t _probe( uint32_t timeout_ms )
{
  esp_err_t err = _format( "CGMI?" );
  if( err == ESP_OK ) err = _transact_once( LW_TXN_GENERIC, timeout_ms );
  return err;
}

static esp_err_t _wait_ready( uint32_t timeout_ms )
{
  const TickType_t start = _now();
  vTaskDelay( pdMS_TO_TICKS( 300 ) );
  while( !_elapsed( start, pdMS_TO_TICKS( timeout_ms ) ) )
  {
    if( _probe( LW_PROBE_TIMEOUT_MS ) == ESP_OK ) return ESP_OK;
    vTaskDelay( pdMS_TO_TICKS( 200 ) );
  }
  return ESP_ERR_TIMEOUT;
}

static void _forget_settings( void )
{
  s_lw.port = 0;
  s_lw.data_rate = UNIT_LORAWAN_KEEP;
  s_lw.trials_known[ 0 ] = s_lw.trials_known[ 1 ] = false;
  s_lw.trials[ 0 ] = 1;
  s_lw.trials[ 1 ] = CONFIG_LORAWAN_CONFIRMED_RETRIES;
}

/* ---- Locking -------------------------------------------------------------- */

static esp_err_t _lock( void )
{
  if( !atomic_load( &s_lw.ready ) ) return ESP_ERR_INVALID_STATE;
  if( xSemaphoreTake( s_lw.lock, pdMS_TO_TICKS( LW_LOCK_TIMEOUT_MS ) ) != pdTRUE )
    return ESP_ERR_TIMEOUT;
  if( !atomic_load( &s_lw.ready ) )
  {
    xSemaphoreGive( s_lw.lock );
    return ESP_ERR_INVALID_STATE;
  }
  return ESP_OK;
}

static void _unlock( void ) { xSemaphoreGive( s_lw.lock ); }

__attribute__( ( format( printf, 1, 2 ) ) ) static esp_err_t
_command( const char *fmt, ... )
{
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  va_list args;
  va_start( args, fmt );
  err = _vformat( fmt, args );
  va_end( args );
  if( err == ESP_OK ) err = _transact( LW_TXN_GENERIC, LW_TIMEOUT_MS, 2 );
  _unlock();
  return err;
}

static esp_err_t _get_ints( const char *name, long *values, size_t count )
{
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  err = _query_ints( name, values, count );
  _unlock();
  return err;
}

static esp_err_t _get_ranged( const char *name, long min, long max, long *out )
{
  esp_err_t err = _get_ints( name, out, 1 );
  if( err == ESP_OK && ( *out < min || *out > max ) )
    err = ESP_ERR_INVALID_RESPONSE;
  return err;
}

static esp_err_t _get_hex_string( const char *name, char *out, size_t hex_len )
{
  if( out == NULL ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  char value[ 40 ] = { 0 };
  err = _run( LW_TIMEOUT_MS, 2, "%s?", name );
  if( err == ESP_OK &&
      ( !_field_copy( name, value, sizeof( value ) ) || !_is_hex( value, hex_len ) ) )
    err = ESP_ERR_INVALID_RESPONSE;
  if( err == ESP_OK ) memcpy( out, value, hex_len + 1 );
  _unlock();
  return err;
}

/* ---- Service task --------------------------------------------------------- */

static void _dispatch_events( void )
{
  static unit_lorawan_event_t event; // only the service task dispatches
  for( ;; )
  {
    if( xSemaphoreTake( s_lw.lock, 0 ) != pdTRUE ) return;
    bool have = s_lw.event_count > 0;
    if( have )
    {
      event = s_lw.events[ s_lw.event_head ];
      s_lw.event_head = ( s_lw.event_head + 1 ) % LW_EVENT_DEPTH;
      s_lw.event_count--;
    }
    unit_lorawan_event_cb_t cb = s_lw.cb;
    void *ctx = s_lw.cb_ctx;
    xSemaphoreGive( s_lw.lock );
    if( !have ) return;
    if( cb ) cb( &event, ctx );
  }
}

static void _service_once( void )
{
  if( xSemaphoreTake( s_lw.lock, 0 ) == pdTRUE )
  {
    _pump();
    if( atomic_load( &s_lw.join_state ) == UNIT_LORAWAN_JOIN_IN_PROGRESS &&
        _elapsed( s_lw.join_start, s_lw.join_ticks ) )
    {
      ESP_LOGW( _TAG, "Join timed out" );
      _set_joined( false, true );
    }
    xSemaphoreGive( s_lw.lock );
  }
  _dispatch_events();
}

static void _service_task( void *arg )
{
  (void)arg;
  while( atomic_load( &s_lw.task_run ) )
  {
    _service_once();
    vTaskDelay( pdMS_TO_TICKS( LW_SERVICE_PERIOD_MS ) );
  }
  atomic_store( &s_lw.task_exited, true );
  vTaskDelete( NULL );
}

/* ---- Lifecycle ------------------------------------------------------------ */

static esp_err_t _probe_baud( uint32_t baud )
{
  esp_err_t err = core2foraws_expports_uart_begin( baud );
  if( err != ESP_OK ) return err;
  bool flushed = false;
  vTaskDelay( pdMS_TO_TICKS( 20 ) );
  core2foraws_expports_uart_read_flush( &flushed );
  s_lw.line_len = 0;
  s_lw.line_overflow = false;
  err = _probe( LW_PROBE_TIMEOUT_MS );
  if( err != ESP_OK ) err = _probe( LW_PROBE_TIMEOUT_MS );
  if( err == ESP_OK ) s_lw.baud = baud;
  return err;
}

static esp_err_t _connect( uint32_t baud )
{
  if( _probe_baud( baud ) == ESP_OK ) return ESP_OK;
  for( size_t i = 0; i < sizeof( _baud_rates ) / sizeof( _baud_rates[ 0 ] ); i++ )
  {
    if( _baud_rates[ i ] == baud || _probe_baud( _baud_rates[ i ] ) != ESP_OK )
      continue;
    ESP_LOGW( _TAG, "Modem answered at %lu baud; switching to %lu",
              (unsigned long)_baud_rates[ i ], (unsigned long)baud );
    if( _format( "CGBR=%lu", (unsigned long)baud ) == ESP_OK &&
        _transact_once( LW_TXN_GENERIC, LW_TIMEOUT_MS ) == ESP_OK &&
        _probe_baud( baud ) == ESP_OK )
      return ESP_OK;
    return _probe_baud( _baud_rates[ i ] );
  }
  return ESP_ERR_NOT_FOUND;
}

static void _read_identity( void )
{
  static const struct
  {
    const char *name;
    size_t offset;
    size_t size;
  } fields[] = {
    { "CGMI", offsetof( unit_lorawan_info_t, manufacturer ),
      sizeof( s_lw.info.manufacturer ) },
    { "CGMM", offsetof( unit_lorawan_info_t, model ), sizeof( s_lw.info.model ) },
    { "CGMR", offsetof( unit_lorawan_info_t, revision ),
      sizeof( s_lw.info.revision ) },
    { "CGSN", offsetof( unit_lorawan_info_t, serial ), sizeof( s_lw.info.serial ) },
  };
  for( size_t i = 0; i < sizeof( fields ) / sizeof( fields[ 0 ] ); i++ )
  {
    char *out = (char *)&s_lw.info + fields[ i ].offset;
    if( _run( LW_TIMEOUT_MS, 2, "%s?", fields[ i ].name ) != ESP_OK ||
        !_field_copy( fields[ i ].name, out, fields[ i ].size ) )
      out[ 0 ] = '\0';
  }
  if( strstr( s_lw.info.manufacturer, "ASR" ) == NULL )
    ESP_LOGW( _TAG, "Unexpected modem manufacturer '%s'", s_lw.info.manufacturer );
}

static void _release( void )
{
  core2foraws_expports_pin_reset( PORT_C_UART_TX_PIN );
  if( s_lw.lock ) vSemaphoreDelete( s_lw.lock );
  memset( &s_lw, 0, sizeof( s_lw ) );
}

esp_err_t unit_lorawan_init( const unit_lorawan_config_t *config )
{
  if( s_lw.lock != NULL ) return ESP_ERR_INVALID_STATE;
  const unit_lorawan_config_t defaults = UNIT_LORAWAN_CONFIG_DEFAULT();
  unit_lorawan_config_t cfg = config ? *config : defaults;
  if( cfg.baud == 0 ) cfg.baud = defaults.baud;
  if( cfg.task_stack_size == 0 ) cfg.task_stack_size = defaults.task_stack_size;
  if( cfg.task_priority == 0 ) cfg.task_priority = defaults.task_priority;
  if( !_baud_supported( cfg.baud ) ) return ESP_ERR_INVALID_ARG;

  memset( &s_lw, 0, sizeof( s_lw ) );
  s_lw.lock = xSemaphoreCreateMutex();
  if( s_lw.lock == NULL ) return ESP_ERR_NO_MEM;
  s_lw.cb = cfg.event_cb;
  s_lw.cb_ctx = cfg.event_ctx;
  s_lw.adr = true;
  _forget_settings();
  atomic_store( &s_lw.join_state, UNIT_LORAWAN_JOIN_IDLE );

  esp_err_t err = _connect( cfg.baud );
  if( err != ESP_OK )
  {
    ESP_LOGE( _TAG, "No ASR650X modem answered on Port C" );
    _release();
    return err;
  }
  _read_identity();
  if( _run( LW_TIMEOUT_MS, 2, "ILOGLVL=0" ) != ESP_OK )
    ESP_LOGW( _TAG, "Could not disable modem logging" );

  atomic_store( &s_lw.task_run, true );
  atomic_store( &s_lw.ready, true );
  if( xTaskCreate( _service_task, "unit_lorawan", cfg.task_stack_size, NULL,
                   cfg.task_priority, &s_lw.task ) != pdPASS )
  {
    _release();
    return ESP_ERR_NO_MEM;
  }
  ESP_LOGI( _TAG, "%s %s %s at %lu baud", s_lw.info.manufacturer,
            s_lw.info.model, s_lw.info.revision, (unsigned long)s_lw.baud );
  return ESP_OK;
}

esp_err_t unit_lorawan_deinit( void )
{
  if( s_lw.lock == NULL ) return ESP_ERR_INVALID_STATE;
  if( s_lw.task != NULL && xTaskGetCurrentTaskHandle() == s_lw.task )
    return ESP_ERR_INVALID_STATE;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  atomic_store( &s_lw.ready, false );
  atomic_store( &s_lw.task_run, false );
  _unlock();
  for( int i = 0; i < 250 && !atomic_load( &s_lw.task_exited ); i++ )
    vTaskDelay( pdMS_TO_TICKS( LW_SERVICE_PERIOD_MS ) );
  if( !atomic_load( &s_lw.task_exited ) )
  {
    // An event callback is still running; freeing now would pull the lock
    // out from under it.
    ESP_LOGE( _TAG, "Service task did not stop; driver left disabled" );
    return ESP_ERR_TIMEOUT;
  }
  _release();
  return ESP_OK;
}

esp_err_t unit_lorawan_set_event_callback( unit_lorawan_event_cb_t cb, void *ctx )
{
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  s_lw.cb = cb;
  s_lw.cb_ctx = ctx;
  _unlock();
  return ESP_OK;
}

esp_err_t unit_lorawan_get_info( unit_lorawan_info_t *info )
{
  if( info == NULL ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  *info = s_lw.info;
  _unlock();
  return ESP_OK;
}

uint32_t unit_lorawan_get_baud( void ) { return s_lw.baud; }

esp_err_t unit_lorawan_set_baud( uint32_t baud )
{
  if( !_baud_supported( baud ) ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  uint32_t previous = s_lw.baud;
  err = _run( LW_TIMEOUT_MS, 1, "CGBR=%lu", (unsigned long)baud );
  if( err == ESP_OK && _probe_baud( baud ) != ESP_OK )
  {
    _probe_baud( previous );
    err = ESP_ERR_INVALID_RESPONSE;
  }
  _unlock();
  return err;
}

esp_err_t unit_lorawan_get_stats( unit_lorawan_stats_t *stats )
{
  if( stats == NULL ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  *stats = s_lw.stats;
  _unlock();
  return ESP_OK;
}

/* ---- Configuration -------------------------------------------------------- */

esp_err_t
unit_lorawan_network_config_from_kconfig( unit_lorawan_network_config_t *cfg )
{
  if( cfg == NULL ) return ESP_ERR_INVALID_ARG;
  *cfg = ( unit_lorawan_network_config_t ){
#ifdef CONFIG_LORAWAN_ABP
    .activation = UNIT_LORAWAN_ABP,
#else
    .activation = UNIT_LORAWAN_OTAA,
#endif
    .dev_eui = CONFIG_LORAWAN_DEVICE_EUI,
    .app_eui = CONFIG_LORAWAN_APP_EUI,
    .app_key = CONFIG_LORAWAN_APP_KEY,
    .dev_addr = CONFIG_LORAWAN_DEV_ADDR,
    .app_skey = CONFIG_LORAWAN_APP_SKEY,
    .nwk_skey = CONFIG_LORAWAN_NWK_SKEY,
    .channel_mask = UNIT_LORAWAN_US915_SUB_BAND( CONFIG_LORAWAN_US915_SUB_BAND ),
    .device_class = UNIT_LORAWAN_CLASS_A,
#ifdef CONFIG_LORAWAN_ADR_ENABLED
    .adr = true,
#else
    .adr = false,
#endif
    .data_rate = CONFIG_LORAWAN_US915_DATA_RATE,
    .tx_power = CONFIG_LORAWAN_TX_POWER_INDEX,
#ifdef CONFIG_LORAWAN_CONFIRMED_UPLINKS
    .confirmed = true,
#else
    .confirmed = false,
#endif
    .confirmed_trials = CONFIG_LORAWAN_CONFIRMED_RETRIES,
    .unconfirmed_trials = 1,
    .app_port = CONFIG_LORAWAN_APP_PORT,
    .rx2_frequency_hz = 0,
    .rx2_data_rate = UNIT_LORAWAN_US915_RX2_DR,
    .rx1_dr_offset = 0,
#ifdef CONFIG_LORAWAN_SAVE_CONFIG
    .save = true,
#else
    .save = false,
#endif
  };
  return ESP_OK;
}

static bool _trials_valid( uint8_t trials )
{
  return trials == UNIT_LORAWAN_KEEP || ( trials >= 1 && trials <= 15 );
}

static esp_err_t _validate_network( const unit_lorawan_network_config_t *cfg )
{
  if( cfg->activation == UNIT_LORAWAN_OTAA )
  {
    if( !_is_hex( cfg->dev_eui, UNIT_LORAWAN_EUI_HEX_LEN ) ) return ESP_ERR_INVALID_ARG;
    if( !_is_hex( cfg->app_eui, UNIT_LORAWAN_EUI_HEX_LEN ) ) return ESP_ERR_INVALID_ARG;
    if( !_is_hex( cfg->app_key, UNIT_LORAWAN_KEY_HEX_LEN ) ) return ESP_ERR_INVALID_ARG;
  }
  else if( cfg->activation == UNIT_LORAWAN_ABP )
  {
    if( !_is_hex( cfg->dev_addr, UNIT_LORAWAN_DEVADDR_HEX_LEN ) ||
        !_is_hex( cfg->app_skey, UNIT_LORAWAN_KEY_HEX_LEN ) ||
        !_is_hex( cfg->nwk_skey, UNIT_LORAWAN_KEY_HEX_LEN ) )
      return ESP_ERR_INVALID_ARG;
  }
  else
  {
    return ESP_ERR_INVALID_ARG;
  }
  if( (unsigned)cfg->device_class > UNIT_LORAWAN_CLASS_C ||
      ( cfg->data_rate != UNIT_LORAWAN_KEEP &&
        cfg->data_rate > UNIT_LORAWAN_US915_DR_MAX ) ||
      ( cfg->tx_power != UNIT_LORAWAN_KEEP && cfg->tx_power > 15 ) ||
      !_trials_valid( cfg->confirmed_trials ) ||
      !_trials_valid( cfg->unconfirmed_trials ) ||
      ( cfg->app_port != UNIT_LORAWAN_KEEP && cfg->app_port > 223 ) )
    return ESP_ERR_INVALID_ARG;
  if( cfg->rx2_frequency_hz != 0 &&
      ( !_us915_frequency( cfg->rx2_frequency_hz ) || cfg->rx2_data_rate > 15 ||
        cfg->rx1_dr_offset > 3 ) )
    return ESP_ERR_INVALID_ARG;
  return ESP_OK;
}

#define LW_STEP( label, ... )                                                  \
  do                                                                           \
  {                                                                            \
    err = _run( LW_TIMEOUT_MS, 2, __VA_ARGS__ );                               \
    if( err != ESP_OK )                                                        \
    {                                                                          \
      ESP_LOGE( _TAG, "Configuring %s failed: %s", label,                     \
                esp_err_to_name( err ) );                                      \
      goto done;                                                               \
    }                                                                          \
  } while( 0 )

esp_err_t unit_lorawan_configure( const unit_lorawan_network_config_t *cfg )
{
  if( cfg == NULL ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _validate_network( cfg );
  if( err != ESP_OK )
  {
    ESP_LOGE( _TAG, "Invalid network configuration (check credential lengths)" );
    return err;
  }
  err = _lock();
  if( err != ESP_OK ) return err;

  // Stop a factory auto-join so credentials are not swapped under it.
  _run( LW_TIMEOUT_MS, 1, "CJOIN=0" );
  atomic_store( &s_lw.join_state, UNIT_LORAWAN_JOIN_IDLE );

  if( cfg->activation == UNIT_LORAWAN_OTAA )
  {
    LW_STEP( "join mode", "CJOINMODE=0" );
    LW_STEP( "DevEUI", "CDEVEUI=%s", cfg->dev_eui );
    LW_STEP( "AppEUI", "CAPPEUI=%s", cfg->app_eui );
    LW_STEP( "AppKey", "CAPPKEY=%s", cfg->app_key );
  }
  else
  {
    LW_STEP( "join mode", "CJOINMODE=1" );
    LW_STEP( "DevAddr", "CDEVADDR=%s", cfg->dev_addr );
    LW_STEP( "AppSKey", "CAPPSKEY=%s", cfg->app_skey );
    LW_STEP( "NwkSKey", "CNWKSKEY=%s", cfg->nwk_skey );
  }
  if( cfg->channel_mask != 0 )
    LW_STEP( "channel mask", "CFREQBANDMASK=%04X", cfg->channel_mask );
  LW_STEP( "UL/DL mode", "CULDLMODE=2" ); // US915 downlinks use 923-928 MHz
  LW_STEP( "work mode", "CWORKMODE=2" );
  LW_STEP( "class", "CCLASS=%d", (int)cfg->device_class );
  LW_STEP( "ADR", "CADR=%d", cfg->adr ? 1 : 0 );
  s_lw.adr = cfg->adr;
  if( cfg->data_rate != UNIT_LORAWAN_KEEP )
  {
    LW_STEP( "data rate", "CDATARATE=%u", cfg->data_rate );
    s_lw.data_rate = cfg->data_rate;
  }
  if( cfg->tx_power != UNIT_LORAWAN_KEEP )
    LW_STEP( "TX power", "CTXP=%u", cfg->tx_power );
  LW_STEP( "uplink type", "CCONFIRM=%d", cfg->confirmed ? 1 : 0 );
  s_lw.default_confirmed = cfg->confirmed;
  if( cfg->confirmed_trials != UNIT_LORAWAN_KEEP )
  {
    LW_STEP( "confirmed trials", "CNBTRIALS=1,%u", cfg->confirmed_trials );
    s_lw.trials[ 1 ] = cfg->confirmed_trials;
    s_lw.trials_known[ 1 ] = true;
  }
  if( cfg->unconfirmed_trials != UNIT_LORAWAN_KEEP )
  {
    LW_STEP( "unconfirmed trials", "CNBTRIALS=0,%u", cfg->unconfirmed_trials );
    s_lw.trials[ 0 ] = cfg->unconfirmed_trials;
    s_lw.trials_known[ 0 ] = true;
  }
  if( cfg->app_port != UNIT_LORAWAN_KEEP && cfg->app_port != 0 )
  {
    LW_STEP( "port", "CAPPPORT=%u", cfg->app_port );
    s_lw.port = cfg->app_port;
  }
  if( cfg->rx2_frequency_hz != 0 )
    LW_STEP( "RX windows", "CRXP=%u,%u,%lu", cfg->rx1_dr_offset,
             cfg->rx2_data_rate, (unsigned long)cfg->rx2_frequency_hz );
  if( cfg->save )
  {
    err = _run( LW_FLASH_TIMEOUT_MS, 1, "CSAVE" );
    if( err != ESP_OK ) goto done;
  }
  if( cfg->activation == UNIT_LORAWAN_ABP ) _set_joined( true, false );

done:
  _unlock();
  return err;
}

esp_err_t unit_lorawan_set_otaa( const char *dev_eui, const char *app_eui,
                                 const char *app_key )
{
  if( !_is_hex( dev_eui, UNIT_LORAWAN_EUI_HEX_LEN ) ||
      !_is_hex( app_eui, UNIT_LORAWAN_EUI_HEX_LEN ) ||
      !_is_hex( app_key, UNIT_LORAWAN_KEY_HEX_LEN ) )
    return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  LW_STEP( "join mode", "CJOINMODE=0" );
  LW_STEP( "DevEUI", "CDEVEUI=%s", dev_eui );
  LW_STEP( "AppEUI", "CAPPEUI=%s", app_eui );
  LW_STEP( "AppKey", "CAPPKEY=%s", app_key );
  atomic_store( &s_lw.join_state, UNIT_LORAWAN_JOIN_IDLE );
done:
  _unlock();
  return err;
}

esp_err_t unit_lorawan_set_abp( const char *dev_addr, const char *app_skey,
                                const char *nwk_skey )
{
  if( !_is_hex( dev_addr, UNIT_LORAWAN_DEVADDR_HEX_LEN ) ||
      !_is_hex( app_skey, UNIT_LORAWAN_KEY_HEX_LEN ) ||
      !_is_hex( nwk_skey, UNIT_LORAWAN_KEY_HEX_LEN ) )
    return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  LW_STEP( "join mode", "CJOINMODE=1" );
  LW_STEP( "DevAddr", "CDEVADDR=%s", dev_addr );
  LW_STEP( "AppSKey", "CAPPSKEY=%s", app_skey );
  LW_STEP( "NwkSKey", "CNWKSKEY=%s", nwk_skey );
  _set_joined( true, false );
done:
  _unlock();
  return err;
}

esp_err_t unit_lorawan_get_dev_eui( char dev_eui[ UNIT_LORAWAN_EUI_HEX_LEN + 1 ] )
{
  return _get_hex_string( "CDEVEUI", dev_eui, UNIT_LORAWAN_EUI_HEX_LEN );
}

esp_err_t unit_lorawan_get_app_eui( char app_eui[ UNIT_LORAWAN_EUI_HEX_LEN + 1 ] )
{
  return _get_hex_string( "CAPPEUI", app_eui, UNIT_LORAWAN_EUI_HEX_LEN );
}

esp_err_t
unit_lorawan_get_dev_addr( char dev_addr[ UNIT_LORAWAN_DEVADDR_HEX_LEN + 1 ] )
{
  return _get_hex_string( "CDEVADDR", dev_addr, UNIT_LORAWAN_DEVADDR_HEX_LEN );
}

esp_err_t unit_lorawan_get_activation( unit_lorawan_activation_t *activation )
{
  if( activation == NULL ) return ESP_ERR_INVALID_ARG;
  long value = 0;
  esp_err_t err = _get_ranged( "CJOINMODE", 0, 1, &value );
  if( err == ESP_OK ) *activation = (unit_lorawan_activation_t)value;
  return err;
}

esp_err_t unit_lorawan_set_channel_mask( uint16_t mask )
{
  if( mask == 0 ) return ESP_ERR_INVALID_ARG;
  return _command( "CFREQBANDMASK=%04X", mask );
}

esp_err_t unit_lorawan_get_channel_mask( uint16_t *mask )
{
  if( mask == NULL ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  err = _run( LW_TIMEOUT_MS, 2, "CFREQBANDMASK?" );
  unsigned long value = 0;
  if( err == ESP_OK &&
      ( !_parse_ulong( _field( "CFREQBANDMASK" ), 16, &value ) || value > 0xFFFF ) )
    err = ESP_ERR_INVALID_RESPONSE;
  if( err == ESP_OK ) *mask = (uint16_t)value;
  _unlock();
  return err;
}

esp_err_t unit_lorawan_set_class( unit_lorawan_class_t device_class )
{
  if( device_class != UNIT_LORAWAN_CLASS_A && device_class != UNIT_LORAWAN_CLASS_C )
    return ESP_ERR_INVALID_ARG;
  return _command( "CCLASS=%d", (int)device_class );
}

esp_err_t unit_lorawan_set_class_b( const unit_lorawan_class_b_t *params )
{
  if( params == NULL || params->periodicity > 7 ) return ESP_ERR_INVALID_ARG;
  if( !params->custom ) return _command( "CCLASS=1,0,%u", params->periodicity );
  if( !_us915_frequency( params->beacon_frequency_hz ) ||
      !_us915_frequency( params->ping_frequency_hz ) ||
      params->beacon_data_rate > 15 || params->ping_data_rate > 15 )
    return ESP_ERR_INVALID_ARG;
  return _command( "CCLASS=1,1,%lu,%u,%lu,%u",
                   (unsigned long)params->beacon_frequency_hz,
                   params->beacon_data_rate,
                   (unsigned long)params->ping_frequency_hz,
                   params->ping_data_rate );
}

esp_err_t unit_lorawan_get_class( unit_lorawan_class_t *device_class )
{
  if( device_class == NULL ) return ESP_ERR_INVALID_ARG;
  long value = 0;
  esp_err_t err = _get_ranged( "CCLASS", 0, 2, &value );
  if( err == ESP_OK ) *device_class = (unit_lorawan_class_t)value;
  return err;
}

esp_err_t unit_lorawan_ping_slot_info_request( uint8_t periodicity )
{
  if( periodicity > 7 ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  err = _run( LW_TIMEOUT_MS, 1, "CPINGSLOTINFOREQ=%u", periodicity );
  _unlock();
  return err;
}

esp_err_t unit_lorawan_set_adr( bool enabled )
{
  esp_err_t err = _command( "CADR=%d", enabled ? 1 : 0 );
  if( err == ESP_OK ) s_lw.adr = enabled;
  return err;
}

esp_err_t unit_lorawan_get_adr( bool *enabled )
{
  if( enabled == NULL ) return ESP_ERR_INVALID_ARG;
  long value = 0;
  esp_err_t err = _get_ranged( "CADR", 0, 1, &value );
  if( err == ESP_OK ) *enabled = value != 0;
  return err;
}

esp_err_t unit_lorawan_set_data_rate( uint8_t data_rate )
{
  if( data_rate > UNIT_LORAWAN_US915_DR_MAX ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _command( "CDATARATE=%u", data_rate );
  if( err == ESP_OK ) s_lw.data_rate = data_rate;
  return err;
}

esp_err_t unit_lorawan_get_data_rate( uint8_t *data_rate )
{
  if( data_rate == NULL ) return ESP_ERR_INVALID_ARG;
  long value = 0;
  esp_err_t err = _get_ranged( "CDATARATE", 0, 15, &value );
  if( err == ESP_OK ) *data_rate = (uint8_t)value;
  return err;
}

size_t unit_lorawan_max_payload( uint8_t data_rate )
{
  return data_rate <= UNIT_LORAWAN_US915_DR_MAX ? _us915_max_payload[ data_rate ]
                                                : 0;
}

esp_err_t unit_lorawan_set_tx_power( uint8_t index )
{
  if( index > 15 ) return ESP_ERR_INVALID_ARG;
  return _command( "CTXP=%u", index );
}

esp_err_t unit_lorawan_get_tx_power( uint8_t *index )
{
  if( index == NULL ) return ESP_ERR_INVALID_ARG;
  long value = 0;
  esp_err_t err = _get_ranged( "CTXP", 0, 15, &value );
  if( err == ESP_OK ) *index = (uint8_t)value;
  return err;
}

esp_err_t unit_lorawan_set_confirmed( bool confirmed )
{
  esp_err_t err = _command( "CCONFIRM=%d", confirmed ? 1 : 0 );
  if( err == ESP_OK ) s_lw.default_confirmed = confirmed;
  return err;
}

esp_err_t unit_lorawan_get_confirmed( bool *confirmed )
{
  if( confirmed == NULL ) return ESP_ERR_INVALID_ARG;
  long value = 0;
  esp_err_t err = _get_ranged( "CCONFIRM", 0, 1, &value );
  if( err == ESP_OK ) *confirmed = value != 0;
  return err;
}

esp_err_t unit_lorawan_set_port( uint8_t port )
{
  if( port < 1 || port > 223 ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _command( "CAPPPORT=%u", port );
  if( err == ESP_OK ) s_lw.port = port;
  return err;
}

esp_err_t unit_lorawan_get_port( uint8_t *port )
{
  if( port == NULL ) return ESP_ERR_INVALID_ARG;
  long value = 0;
  esp_err_t err = _get_ranged( "CAPPPORT", 1, 223, &value );
  if( err == ESP_OK ) *port = (uint8_t)value;
  return err;
}

esp_err_t unit_lorawan_set_trials( bool confirmed, uint8_t trials )
{
  if( trials < 1 || trials > 15 ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _command( "CNBTRIALS=%d,%u", confirmed ? 1 : 0, trials );
  if( err == ESP_OK )
  {
    s_lw.trials[ confirmed ] = trials;
    s_lw.trials_known[ confirmed ] = true;
  }
  return err;
}

esp_err_t unit_lorawan_get_trials( bool confirmed, uint8_t *trials )
{
  if( trials == NULL ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  long value[ 2 ] = { 0 };
  err = _query_ints( "CNBTRIALS", value, 2 );
  // The inquiry reports one <MType>,<value> pair, which may be the other type.
  bool matches = err == ESP_OK && value[ 0 ] == ( confirmed ? 1 : 0 ) &&
                 value[ 1 ] >= 1 && value[ 1 ] <= 15;
  if( matches )
  {
    *trials = (uint8_t)value[ 1 ];
  }
  else if( ( err == ESP_OK || err == ESP_ERR_INVALID_RESPONSE ) &&
           s_lw.trials_known[ confirmed ] )
  {
    *trials = s_lw.trials[ confirmed ];
    err = ESP_OK;
  }
  else if( err == ESP_OK )
  {
    err = ESP_ERR_INVALID_RESPONSE;
  }
  _unlock();
  return err;
}

esp_err_t unit_lorawan_set_rx_params( const unit_lorawan_rx_params_t *params )
{
  if( params == NULL || params->rx1_dr_offset > 3 || params->rx2_data_rate > 15 ||
      !_us915_frequency( params->rx2_frequency_hz ) )
    return ESP_ERR_INVALID_ARG;
  return _command( "CRXP=%u,%u,%lu", params->rx1_dr_offset,
                   params->rx2_data_rate,
                   (unsigned long)params->rx2_frequency_hz );
}

esp_err_t unit_lorawan_get_rx_params( unit_lorawan_rx_params_t *params )
{
  if( params == NULL ) return ESP_ERR_INVALID_ARG;
  long value[ 3 ] = { 0 };
  esp_err_t err = _get_ints( "CRXP", value, 3 );
  if( err == ESP_OK && ( value[ 0 ] < 0 || value[ 0 ] > 15 || value[ 1 ] < 0 ||
                         value[ 1 ] > 15 || value[ 2 ] < 0 ) )
    err = ESP_ERR_INVALID_RESPONSE;
  if( err == ESP_OK )
  {
    params->rx1_dr_offset = (uint8_t)value[ 0 ];
    params->rx2_data_rate = (uint8_t)value[ 1 ];
    params->rx2_frequency_hz = (uint32_t)value[ 2 ];
  }
  return err;
}

esp_err_t unit_lorawan_set_rx1_delay( uint8_t seconds )
{
  if( seconds < 1 || seconds > 15 ) return ESP_ERR_INVALID_ARG;
  return _command( "CRX1DELAY=%u", seconds );
}

esp_err_t unit_lorawan_get_rx1_delay( uint8_t *seconds )
{
  if( seconds == NULL ) return ESP_ERR_INVALID_ARG;
  long value = 0;
  esp_err_t err = _get_ranged( "CRX1DELAY", 0, 15, &value );
  if( err == ESP_OK ) *seconds = (uint8_t)value;
  return err;
}

esp_err_t unit_lorawan_set_report_mode( bool periodic, uint16_t interval_s )
{
  if( !periodic ) return _command( "CRM=0" );
  if( interval_s == 0 ) return ESP_ERR_INVALID_ARG;
  return _command( "CRM=1,%u", interval_s );
}

esp_err_t unit_lorawan_save( void )
{
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  err = _run( LW_FLASH_TIMEOUT_MS, 1, "CSAVE" );
  _unlock();
  return err;
}

esp_err_t unit_lorawan_restore_defaults( void )
{
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  err = _run( LW_FLASH_TIMEOUT_MS, 1, "CRESTORE" );
  if( err == ESP_OK )
  {
    _forget_settings();
    s_lw.adr = true;
    atomic_store( &s_lw.join_state, UNIT_LORAWAN_JOIN_IDLE );
  }
  _unlock();
  return err;
}

/* ---- Join and data -------------------------------------------------------- */

esp_err_t unit_lorawan_join( const unit_lorawan_join_params_t *params )
{
  const unit_lorawan_join_params_t defaults = UNIT_LORAWAN_JOIN_PARAMS_DEFAULT();
  const unit_lorawan_join_params_t *p = params ? params : &defaults;
  if( p->period_s < 7 || p->max_attempts < 1 || p->max_attempts > 256 )
    return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;

  uint32_t timeout_ms = p->timeout_ms;
  if( timeout_ms == 0 )
    timeout_ms = ( (uint32_t)p->period_s * p->max_attempts + 30U ) * 1000U;
  int previous = atomic_exchange( &s_lw.join_state, UNIT_LORAWAN_JOIN_IN_PROGRESS );
  s_lw.join_start = _now();
  s_lw.join_ticks = pdMS_TO_TICKS( timeout_ms );
  err = _run( LW_TIMEOUT_MS, 1, "CJOIN=1,%d,%u,%u", p->auto_join ? 1 : 0,
              p->period_s, p->max_attempts );
  if( err != ESP_OK )
  {
    int expected = UNIT_LORAWAN_JOIN_IN_PROGRESS;
    atomic_compare_exchange_strong( &s_lw.join_state, &expected, previous );
  }
  _unlock();
  return err;
}

esp_err_t unit_lorawan_join_stop( void )
{
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  err = _run( LW_TIMEOUT_MS, 1, "CJOIN=0" );
  if( err == ESP_OK )
  {
    int expected = UNIT_LORAWAN_JOIN_IN_PROGRESS;
    atomic_compare_exchange_strong( &s_lw.join_state, &expected,
                                    UNIT_LORAWAN_JOIN_IDLE );
  }
  _unlock();
  return err;
}

unit_lorawan_join_state_t unit_lorawan_get_join_state( void )
{
  return (unit_lorawan_join_state_t)atomic_load( &s_lw.join_state );
}

esp_err_t unit_lorawan_get_status( unit_lorawan_status_t *status )
{
  if( status == NULL ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  long value = 0;
  err = _query_ints( "CSTATUS", &value, 1 );
  if( err == ESP_OK && ( value < 0 || value > 8 ) ) err = ESP_ERR_INVALID_RESPONSE;
  if( err == ESP_OK )
  {
    *status = (unit_lorawan_status_t)value;
    // Recovers the join result if the +CJOIN line was lost.
    if( value == UNIT_LORAWAN_STATUS_JOIN_OK &&
        atomic_load( &s_lw.join_state ) == UNIT_LORAWAN_JOIN_IN_PROGRESS )
      _set_joined( true, false );
  }
  _unlock();
  return err;
}

esp_err_t unit_lorawan_wait_joined( uint32_t timeout_ms )
{
  if( !atomic_load( &s_lw.ready ) ) return ESP_ERR_INVALID_STATE;
  const TickType_t start = _now();
  TickType_t last_poll = start;
  for( ;; )
  {
    switch( atomic_load( &s_lw.join_state ) )
    {
    case UNIT_LORAWAN_JOIN_JOINED:
      return ESP_OK;
    case UNIT_LORAWAN_JOIN_FAILED:
      return ESP_FAIL;
    case UNIT_LORAWAN_JOIN_IDLE:
      return ESP_ERR_INVALID_STATE;
    default:
      break;
    }
    if( _elapsed( start, pdMS_TO_TICKS( timeout_ms ) ) ) return ESP_ERR_TIMEOUT;
    if( _elapsed( last_poll, pdMS_TO_TICKS( LW_JOIN_POLL_MS ) ) )
    {
      unit_lorawan_status_t status;
      unit_lorawan_get_status( &status );
      last_poll = _now();
    }
    vTaskDelay( pdMS_TO_TICKS( 50 ) );
  }
}

esp_err_t unit_lorawan_send( const uint8_t *data, size_t length,
                             const unit_lorawan_tx_params_t *params,
                             unit_lorawan_tx_result_t *result )
{
  if( result ) memset( result, 0, sizeof( *result ) );
  if( ( data == NULL && length > 0 ) || length > UNIT_LORAWAN_MAX_UPLINK ||
      ( params && ( params->trials > 15 || params->port > 223 ) ) )
    return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;

  bool confirmed = params ? params->confirmed : s_lw.default_confirmed;
  if( !s_lw.adr && s_lw.data_rate <= UNIT_LORAWAN_US915_DR_MAX &&
      length > _us915_max_payload[ s_lw.data_rate ] )
  {
    err = ESP_ERR_INVALID_SIZE;
    goto done;
  }
  if( params && params->port != 0 && params->port != s_lw.port )
  {
    err = _run( LW_TIMEOUT_MS, 2, "CAPPPORT=%u", params->port );
    if( err != ESP_OK ) goto done;
    s_lw.port = params->port;
  }

  uint8_t trials = params && params->trials ? params->trials
                                            : s_lw.trials[ confirmed ];
  int prefix = snprintf( s_lw.wire, sizeof( s_lw.wire ), "AT+DTRX=%d,%u,%u,",
                         confirmed ? 1 : 0, trials, (unsigned)( length * 2 ) );
  size_t position = (size_t)prefix;
  for( size_t i = 0; i < length; i++ )
  {
    s_lw.wire[ position++ ] = _hex_digits[ data[ i ] >> 4 ];
    s_lw.wire[ position++ ] = _hex_digits[ data[ i ] & 0x0F ];
  }
  s_lw.wire[ position++ ] = '\r';
  s_lw.wire[ position ] = '\0';
  s_lw.wire_len = position;

  uint32_t timeout_ms = params && params->timeout_ms
                            ? params->timeout_ms
                            : 5000U + 7000U * trials;
  s_lw.stats.uplinks++;
  err = _transact( LW_TXN_UPLINK, timeout_ms, 1 );
  if( err == ESP_OK && confirmed ) s_lw.txn.tx.acked = true;
  if( err == ESP_ERR_INVALID_STATE && !s_lw.test_mode )
    atomic_store( &s_lw.join_state, UNIT_LORAWAN_JOIN_IDLE );
  if( err == ESP_OK &&
      atomic_load( &s_lw.join_state ) != UNIT_LORAWAN_JOIN_JOINED )
    _set_joined( true, false );
  if( result && !s_lw.test_mode ) *result = s_lw.txn.tx;

done:
  _unlock();
  return err;
}

esp_err_t unit_lorawan_read_rx_buffer( uint8_t *data, size_t capacity,
                                       size_t *length )
{
  if( length == NULL || ( data == NULL && capacity > 0 ) ) return ESP_ERR_INVALID_ARG;
  *length = 0;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  err = _run( LW_TIMEOUT_MS, 1, "DRX?" );
  const char *value = err == ESP_OK ? _field( "DRX" ) : NULL;
  const char *payload = value ? strchr( value, ',' ) : NULL;
  if( payload )
  {
    payload++;
    size_t hex_length = strcspn( payload, "\n" );
    if( hex_length & 1U ) err = ESP_ERR_INVALID_RESPONSE;
    else if( hex_length / 2 > capacity ) err = ESP_ERR_INVALID_SIZE;
    for( size_t i = 0; err == ESP_OK && i < hex_length / 2; i++ )
    {
      int high = _hex_value( payload[ 2 * i ] );
      int low = _hex_value( payload[ 2 * i + 1 ] );
      if( high < 0 || low < 0 ) err = ESP_ERR_INVALID_RESPONSE;
      else data[ i ] = (uint8_t)( ( high << 4 ) | low );
    }
    if( err == ESP_OK ) *length = hex_length / 2;
  }
  _unlock();
  return err;
}

/* ---- Diagnostics ---------------------------------------------------------- */

esp_err_t unit_lorawan_link_check( unit_lorawan_link_check_mode_t mode )
{
  if( (unsigned)mode > UNIT_LORAWAN_LINK_CHECK_EVERY_UPLINK )
    return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  err = _run( LW_TIMEOUT_MS, 1, "CLINKCHECK=%d", (int)mode );
  _unlock();
  return err;
}

esp_err_t unit_lorawan_get_last_link_check( unit_lorawan_link_check_t *out )
{
  if( out == NULL ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  if( s_lw.link_check_valid ) *out = s_lw.link_check;
  else err = ESP_ERR_NOT_FOUND;
  _unlock();
  return err;
}

esp_err_t unit_lorawan_get_channel_rssi( uint8_t group, int16_t rssi_dbm[ 8 ] )
{
  if( rssi_dbm == NULL || group > 15 ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  err = _run( LW_RSSI_TIMEOUT_MS, 2, "CRSSI %u?", group );
  unsigned seen = 0;
  for( const char *line = s_lw.txn.reply; err == ESP_OK && *line; )
  {
    char *end = NULL;
    long channel = strtol( line, &end, 10 );
    if( end != line && *end == ':' && channel >= 0 && channel < 8 )
    {
      const char *value_text = end + 1;
      long value = strtol( value_text, &end, 10 );
      if( end != value_text && value >= INT16_MIN && value <= INT16_MAX )
      {
        rssi_dbm[ channel ] = (int16_t)value;
        seen |= 1U << channel;
      }
    }
    const char *next = strchr( line, '\n' );
    if( next == NULL ) break;
    line = next + 1;
  }
  if( err == ESP_OK && seen != 0xFF ) err = ESP_ERR_INVALID_RESPONSE;
  _unlock();
  return err;
}

esp_err_t unit_lorawan_get_battery_level( uint8_t *level )
{
  if( level == NULL ) return ESP_ERR_INVALID_ARG;
  long value = 0;
  esp_err_t err = _get_ranged( "CBL", 0, 255, &value );
  if( err == ESP_OK ) *level = (uint8_t)value;
  return err;
}

esp_err_t unit_lorawan_set_log_level( uint8_t level )
{
  if( level > 5 ) return ESP_ERR_INVALID_ARG;
  return _command( "ILOGLVL=%u", level );
}

/* ---- Multicast ------------------------------------------------------------ */

esp_err_t unit_lorawan_multicast_add( const unit_lorawan_multicast_t *group )
{
  if( group == NULL || !_is_hex( group->dev_addr, UNIT_LORAWAN_DEVADDR_HEX_LEN ) ||
      !_is_hex( group->app_skey, UNIT_LORAWAN_KEY_HEX_LEN ) ||
      !_is_hex( group->nwk_skey, UNIT_LORAWAN_KEY_HEX_LEN ) ||
      ( group->class_b && ( group->periodicity > 7 || group->data_rate > 15 ) ) )
    return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  if( group->class_b )
    err = _run( LW_TIMEOUT_MS, 1, "CADDMUTICAST=%s,%s,%s,%u,%u", group->dev_addr,
                group->app_skey, group->nwk_skey, group->periodicity,
                group->data_rate );
  else
    err = _run( LW_TIMEOUT_MS, 1, "CADDMUTICAST=%s,%s,%s", group->dev_addr,
                group->app_skey, group->nwk_skey );
  _unlock();
  return err;
}

esp_err_t unit_lorawan_multicast_remove( const char *dev_addr )
{
  if( !_is_hex( dev_addr, UNIT_LORAWAN_DEVADDR_HEX_LEN ) ) return ESP_ERR_INVALID_ARG;
  return _command( "CDELMUTICAST=%s", dev_addr );
}

esp_err_t unit_lorawan_multicast_count( uint8_t *count )
{
  if( count == NULL ) return ESP_ERR_INVALID_ARG;
  long value = 0;
  esp_err_t err = _get_ranged( "CNUMMUTICAST", 0, 255, &value );
  if( err == ESP_OK ) *count = (uint8_t)value;
  return err;
}

/* ---- Power and reset ------------------------------------------------------ */

esp_err_t unit_lorawan_set_low_power( bool enabled )
{
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  // CLPM=0 can be misread while the modem wakes, so it gets extra attempts.
  if( enabled ) err = _run( LW_TIMEOUT_MS, 1, "CLPM=1" );
  else
  {
    s_lw.wake_next = true;
    err = _run( LW_TIMEOUT_MS, 3, "CLPM=0" );
  }
  if( err == ESP_OK ) s_lw.low_power = enabled;
  _unlock();
  return err;
}

esp_err_t unit_lorawan_reboot( unit_lorawan_reboot_mode_t mode )
{
  if( mode != UNIT_LORAWAN_REBOOT_NOW && mode != UNIT_LORAWAN_REBOOT_AFTER_TX &&
      mode != UNIT_LORAWAN_REBOOT_BOOTLOADER )
    return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  bool was_test_mode = s_lw.test_mode;
  s_lw.test_mode = false; // a reboot is the only recovery from test loops
  err = _run( LW_TIMEOUT_MS, 1, "IREBOOT=%d", (int)mode );
  if( err == ESP_OK && mode == UNIT_LORAWAN_REBOOT_BOOTLOADER )
  {
    s_lw.test_mode = true;
  }
  else if( err == ESP_OK )
  {
    atomic_store( &s_lw.join_state, UNIT_LORAWAN_JOIN_IDLE );
    s_lw.line_len = 0;
    s_lw.wake_next = s_lw.low_power;
    err = _wait_ready( mode == UNIT_LORAWAN_REBOOT_AFTER_TX ? LW_BOOT_AFTER_TX_MS
                                                            : LW_BOOT_TIMEOUT_MS );
    if( err == ESP_OK )
    {
      _run( LW_TIMEOUT_MS, 2, "ILOGLVL=0" );
      _forget_settings();
    }
  }
  else
  {
    s_lw.test_mode = was_test_mode;
  }
  _unlock();
  return err;
}

esp_err_t unit_lorawan_protect_keys_irreversible( const char *key )
{
  if( !_is_hex( key, UNIT_LORAWAN_KEY_HEX_LEN ) ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  err = _run( LW_FLASH_TIMEOUT_MS, 1, "CKEYSPROTECT=%s", key );
  _unlock();
  return err;
}

esp_err_t unit_lorawan_keys_protected( bool *protected_keys )
{
  if( protected_keys == NULL ) return ESP_ERR_INVALID_ARG;
  long value = 0;
  esp_err_t err = _get_ints( "CKEYSPROTECT", &value, 1 );
  if( err == ESP_OK ) *protected_keys = value != 0;
  return err;
}

/* ---- Factory tests -------------------------------------------------------- */

__attribute__( ( format( printf, 2, 3 ) ) ) static esp_err_t
_test_command( bool latch, const char *fmt, ... )
{
  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  va_list args;
  va_start( args, fmt );
  err = _vformat( fmt, args );
  va_end( args );
  if( err == ESP_OK ) err = _transact( LW_TXN_TEST, LW_TIMEOUT_MS, 1 );
  if( err == ESP_OK && latch )
  {
    s_lw.test_mode = true;
    atomic_store( &s_lw.join_state, UNIT_LORAWAN_JOIN_IDLE );
    ESP_LOGW( _TAG, "Modem in test mode; power-cycle to recover" );
  }
  _unlock();
  return err;
}

esp_err_t unit_lorawan_test_rx( uint32_t frequency_hz, uint8_t data_rate )
{
  if( frequency_hz < 150000000UL || frequency_hz > 960000000UL || data_rate > 5 )
    return ESP_ERR_INVALID_ARG;
  return _test_command( true, "CRX=%lu,%u", (unsigned long)frequency_hz, data_rate );
}

esp_err_t unit_lorawan_test_tx( uint32_t frequency_hz, uint8_t data_rate,
                                uint8_t power_dbm )
{
  if( !_us915_frequency( frequency_hz ) || data_rate > 5 || power_dbm > 22 )
    return ESP_ERR_INVALID_ARG;
  return _test_command( true, "CTX=%lu,%u,%u", (unsigned long)frequency_hz,
                        data_rate, power_dbm );
}

esp_err_t unit_lorawan_test_tx_cw( uint32_t frequency_hz, uint8_t power_dbm,
                                   uint8_t pa_option )
{
  if( !_us915_frequency( frequency_hz ) || power_dbm > 22 || pa_option > 3 )
    return ESP_ERR_INVALID_ARG;
  return _test_command( true, "CTXCW=%lu,%u,%u", (unsigned long)frequency_hz,
                        power_dbm, pa_option );
}

esp_err_t unit_lorawan_test_sleep( uint8_t mode )
{
  if( mode > 2 ) return ESP_ERR_INVALID_ARG;
  // Mode 1 wakes only on the set_b pin, which Port C does not carry.
  esp_err_t err = _test_command( mode == 1, "CSLEEP=%u", mode );
  if( err == ESP_OK && mode != 1 && _lock() == ESP_OK )
  {
    if( mode == 0 )
    {
      s_lw.sleeping = true;
      s_lw.sleep_until = _now() + pdMS_TO_TICKS( LW_SLEEP_TIMER_MS );
    }
    else
    {
      s_lw.wake_next = true;
    }
    _unlock();
  }
  return err;
}

esp_err_t unit_lorawan_test_mcu( uint8_t mode )
{
  if( mode > 3 ) return ESP_ERR_INVALID_ARG;
  return _test_command( true, "CMCU=%u", mode );
}

esp_err_t unit_lorawan_test_standby( uint8_t mode )
{
  if( mode > 1 ) return ESP_ERR_INVALID_ARG;
  esp_err_t err = _test_command( false, "CSTDBY=%u", mode );
  if( err == ESP_OK && _lock() == ESP_OK )
  {
    s_lw.wake_next = true;
    _unlock();
  }
  return err;
}

/* ---- Raw access ----------------------------------------------------------- */

esp_err_t unit_lorawan_command( const char *command, char *reply,
                                size_t reply_size, uint32_t timeout_ms )
{
  if( command == NULL || *command == '\0' || ( reply == NULL && reply_size ) ||
      ( reply != NULL && reply_size == 0 ) )
    return ESP_ERR_INVALID_ARG;
  for( const char *c = command; *c; c++ )
    if( *c < 0x20 || *c > 0x7E ) return ESP_ERR_INVALID_ARG;
  if( reply ) reply[ 0 ] = '\0';

  esp_err_t err = _lock();
  if( err != ESP_OK ) return err;
  err = _format( "%s", command );
  if( err == ESP_OK )
    err = _transact( LW_TXN_GENERIC, timeout_ms ? timeout_ms : LW_TIMEOUT_MS, 1 );
  if( reply && ( err == ESP_OK || err == ESP_FAIL ) )
  {
    size_t length = s_lw.txn.reply_len;
    if( length && s_lw.txn.reply[ length - 1 ] == '\n' ) length--;
    if( length >= reply_size )
    {
      length = reply_size - 1;
      if( err == ESP_OK ) err = ESP_ERR_INVALID_SIZE;
    }
    memcpy( reply, s_lw.txn.reply, length );
    reply[ length ] = '\0';
  }
  if( err == ESP_OK && s_lw.txn.truncated ) err = ESP_ERR_INVALID_SIZE;
  _unlock();
  return err;
}
