# M5Stack Unit LoRaWAN915 ESP-IDF Component

Driver for the [M5Stack Unit LoRaWAN915](https://docs.m5stack.com/en/unit/lorawan915) (U115, ASR6501). It connects to Port C of the Core2 for AWS IoT Kit through the BSP UART (GPIO 14 TX, GPIO 13 RX, 115200 8N1). The unit supports US915 only (North America and the other US915 countries), not AU915.

## Quick start

Set credentials under `Component config -> Unit LoRaWAN (US915)`. Keep them in the local, untracked `sdkconfig`.

```c
static void on_event( const unit_lorawan_event_t *event, void *ctx )
{
    switch( event->id )
    {
    case UNIT_LORAWAN_EVENT_JOINED:      /* ... */ break;
    case UNIT_LORAWAN_EVENT_JOIN_FAILED: /* event->join_timed_out */ break;
    case UNIT_LORAWAN_EVENT_DOWNLINK:    /* event->downlink.port/data/length */ break;
    case UNIT_LORAWAN_EVENT_LINK_CHECK:  /* event->link_check */ break;
    }
}

unit_lorawan_config_t config = UNIT_LORAWAN_CONFIG_DEFAULT();
config.event_cb = on_event;
ESP_ERROR_CHECK( unit_lorawan_init( &config ) );

unit_lorawan_network_config_t network;
unit_lorawan_network_config_from_kconfig( &network );
ESP_ERROR_CHECK( unit_lorawan_configure( &network ) );
ESP_ERROR_CHECK( unit_lorawan_join( NULL ) );
ESP_ERROR_CHECK( unit_lorawan_wait_joined( 120000 ) );

unit_lorawan_tx_result_t result;
const uint8_t payload[] = { 0x01, 0x02 };
esp_err_t err = unit_lorawan_send( payload, sizeof( payload ), NULL, &result );
```

## Behavior

- **Events.** A service task reads the UART every 20 ms while no command is running. Join results, downlinks, and link-check answers are delivered even between commands. Callbacks run on that task and may call any driver function. Up to 8 events are queued; the oldest is dropped on overflow and counted in `unit_lorawan_get_stats()`.
- **Commands.** Every call is thread safe and serialized. Queries and plain setters are retried once on timeout. Uplinks, joins, saves, reboots, and test commands are never resent.
- **Uplinks.** `unit_lorawan_send()` blocks until the modem reports `OK+SENT` or an error. Its error codes say why a send failed: not joined, modem busy, too long for the data rate, or no ACK.
- **Parsing.** The reply parser ignores command echo and accepts both `+CMD:` and `+CMD=`. It also accepts the full-width commas shown in the vendor's link-check example. `OK+RECV` TYPE, PORT, and LEN are read as the two-digit hex bytes the modem prints; the DATA length is authoritative.
- **Baud recovery.** At init the configured baud is probed first, then 57600, 38400, 19200, and 9600. A modem found at another rate is moved back with `AT+CGBR`.
- **Flash wear.** `unit_lorawan_configure()` rewrites the full configuration at every start, so `AT+CSAVE` is optional (`LORAWAN_SAVE_CONFIG`).
- **Low power.** With `unit_lorawan_set_low_power( true )`, each command is preceded by the documented wake sequence `00 00 00 00 0D 0A`.

## North America notes

- The default channel mask is sub-band 2 (channels 8-15, 903.9-905.3 MHz), which TTN, Helium, and most North American gateways use.
- The client-side payload limits follow US915: DR0 11 B, DR1 53 B, DR2 125 B, DR3/DR4 242 B. DR0 keeps airtime under the FCC 400 ms dwell limit only at 11 bytes. With ADR on, the modem enforces the limit and the driver returns `ESP_ERR_INVALID_SIZE`.
- RX2 and Class B frequencies are limited to 902-928 MHz. RF test transmissions (`unit_lorawan_test_tx*()`) are also limited to 902-928 MHz.
- Confirmed uplinks are off by default. TTN's fair-use policy allows about 10 downlinks per device per day, and every ACK counts as one. Use periodic link checks to detect lost coverage.

## Hazards

- `unit_lorawan_test_rx()`, `unit_lorawan_test_tx()`, `unit_lorawan_test_tx_cw()`, and `unit_lorawan_test_mcu()` put the modem into a loop that ignores AT commands. The driver refuses further commands until `unit_lorawan_reboot()` succeeds; usually the unit must be power-cycled. Port C has no reset line.
- `unit_lorawan_protect_keys_irreversible()` permanently locks the credentials.
- `unit_lorawan_command()` sends raw commands without updating driver state.

See [datasheets/ASR650X.md](datasheets/ASR650X.md) and [datasheets/schema.yml](datasheets/schema.yml) for the protocol reference.

## License

Licensed under the [Apache License 2.0](LICENSE). Redistributions must retain the attribution in [NOTICE](NOTICE).
