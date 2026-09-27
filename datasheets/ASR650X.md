# Firmware Driver Implementation Specification

## 2026-09-26 Driver 1.0.0 And Firmware Boundaries

The checked-in v4.0 PDF is the reproducible primary source. Statements
attributed below to the externally reviewed v4.3 document are version-specific
and do not override v4.0 silently. Record the installed modem's `CGMR?`
response when qualifying a deployment.

Driver 1.0.0 has a typed API for every command in section 5.2 except
`CFREQLIST`, which v4.0 marks unsupported; `unit_lorawan_command()` covers it.
Firmware may reject optional commands; a generic AT reference is not proof of
support on a particular modem image.

A service task polls the UART while no command holds the lock, so join
results, downlinks, and link-check answers arrive between commands. Unsolicited
lines are recognized before transaction handling, so an `OK+RECV` during a
query still becomes an event. Events are dispatched after the lock is released,
so callbacks may issue commands.

Exact result lines are parsed; `NOTOK` is not success. Command echo is
ignored. Queries and plain setters retry once on timeout. `DTRX`, `CJOIN`,
`CLINKCHECK`, `CSAVE`, `CRESTORE`, `IREBOOT`, `DRX?`, multicast, key
protection, and test commands are never resent. CR/LF and non-printable bytes
are rejected in raw commands. Credential values are never logged. Init never
writes flash; `unit_lorawan_configure()` rewrites the full configuration at
each start, so `CSAVE` is optional.

`OK+SENT` completes an uplink after a 300 ms grace period for a trailing
`OK+RECV`. For confirmed uplinks, `OK+SENT` implies an ACK because failure is
reported as `ERR+SENT`. A successful uplink marks the session joined;
`ERR+SEND:0` marks it not joined. `CSTATUS` 04 recovers a join result whose
`+CJOIN:OK` line was lost. An accepted send is still not proof of delivery
for unconfirmed uplinks.

Generic v4.0 DR0..DR5/SF12..SF7 and CN470 power examples are not US915 tables.
The US915 API uses DR0..4 and its regional payload bounds. Neither
CFREQBANDMASK nor a radio test frequency command is proof of a regional
firmware change. RX2, Class B, and TX-test frequencies are limited to
902-928 MHz. Never run continuous TX, bootloader entry, irreversible key
protection, or factory restore as an automatic probe. The board's published
maximum is +21 dBm, not a regional 30 dBm EIRP table entry.

## M5Stack Unit LoRaWAN915 (SKU U115) using ASR6501 AT-command modem

## Source Evidence and Version Scope

The checked-in authoritative source is `ASR650XATCommandIntroduction-20190605_1695629825.pdf`, ASR650X AT Command Introduction v4.0, SHA-256 `29d0d3d6f6325108eecd2a2f3273dbfcac9e4134960115b30ed1fa52230cd905`. On 2026-08-01 the official [M5Stack product page](https://docs.m5stack.com/en/unit/lorawan915) linked ASR6501/ASR6502 AT reference v4.3 (reviewed SHA-256 `5af461ba120041e04776b183878a7280873a07c51c43da0d880ab1ab345e61e9`). Current framing, including `OK+RECV:TYPE,PORT,LEN,DATA`, was rechecked against v4.3. On 2026-09-26 all 49 v4.0 command sections (5.2.1-5.2.49) were rechecked against text extracted with mupdf. See `schema.yml`.

This specification is a self-contained firmware implementation reference for the M5Stack Unit LoRaWAN915 module. It is intended to support automatic generation of MCU drivers, HAL integrations, BSPs, serial protocol implementations, and test harnesses without requiring the original AT-command document. The module exposes a UART host interface and is controlled entirely through ASCII AT commands rather than a host-visible register map. The command set, UART framing, join parameters, data-transfer formats, status model, power modes, and test modes are defined by the ASR650X AT command interface .

---

## 1. Purpose

This document defines the software-facing interface of the Unit LoRaWAN915 module as an AT-command-controlled LoRaWAN modem. It includes:

* host UART interface requirements
* command syntax and responses
* all documented commands and parameters
* host-visible state and status behavior
* initialization and runtime sequences
* safety rules for generated code

The module is intended to let an external MCU act as the terminal equipment (TE) and control the LoRa modem over UART as the terminal adaptor/mobile terminal side .

---

## 2. Device Overview

### 2.1 Function

Unit LoRaWAN915 is a 915 MHz LoRaWAN communication module based on ASR6501. It supports long-range, low-power wireless communication and exposes the LoRaWAN stack through a UART AT-command interface. The module supports LoRaWAN v1.0.1, receive sensitivity down to `-137 dBm` at `SF=12/BW=125KHz`, and maximum transmit power of `+21 dBm` according to the official M5Stack product information and linked AT reference.

### 2.2 Intended application class

* low-power IoT nodes
* remote telemetry
* environmental monitoring
* gateway-connected LoRaWAN end devices

### 2.3 Major capabilities

* LoRaWAN join using OTAA or ABP 
* LoRaWAN Class A, B, and C configuration 
* uplink and downlink data transport via AT commands 
* ADR enable/disable 
* link check, RSSI inquiry, multicast configuration, RX window configuration, RX1 delay configuration 
* low-power and radio test modes 

### 2.4 Internal functional subsystems relevant to software

* UART AT parser
* LoRaWAN MAC stack
* join/authentication engine
* uplink/downlink buffers
* persistent MAC configuration store in EEPROM/FLASH 
* low-power control
* radio test engine
* bootloader entry via reboot mode `7` 

### 2.5 Product-level constraints

* default band support: `US915`
* product note: supports `US915` by default and does **not** support `AU915`
* nominal host UART setting: `115200`, `8 data bits`, `1 stop bit`, `no parity` 

---

## 3. Communication Interfaces

## 3.1 Host communication interface

### UART

The module is controlled over UART using ASCII AT commands. The documented console parameters are:

* baud rate: `115200`
* data bits: `8`
* stop bits: `1`
* parity/check bit: `0` (no parity) 

### Host framing

Request format:

`AT+<CMD>[OP][para-1,para-2,...,para-n]<\r>`

where:

* `AT+` = command prefix
* `CMD` = instruction string
* `OP` may be:

  * `=` set
  * `?` inquire current setting
  * empty = execute
  * `=?` test/query argument format
* command terminator = carriage return, ASCII `0x0D` 

Reply formats:

* `\r\n[+CMD:][para-1,para-2,...,para-n]\r\n`
* `\r\n<STATUS>\r\n`
* or both

Status values:

* `OK`
* `ERROR`
* `+CME ERROR:<err>` 

`<err>` numbers are not listed in the PDF, which refers to the 3GPP
"AT command set for User Equipment (UE)".

The separator in value replies is inconsistent. The format rows show
`+CMD:<value>`, but the `CGMI`, `CGMM`, `CGMR`, `CGSN`, `CDEVEUI`, `CBL`,
and `CSTATUS` examples show `+CMD=<value>`. Parsers must accept both.

### Echo/backspace behavior

* command echo is supported
* backspace is not supported
* history shortcut key is not supported 

## 3.2 Physical connector interface

From user-supplied product info:

HY2.0-4P / Port C mapping:

* Black: `GND`
* Red: `5V`
* Yellow: `UART_RX`
* White: `UART_TX`

From the supplied schematic:

* board input is `+5V`
* onboard regulator AMS1117-3.3 generates `+3.3V`
* UART lines to the radio module use series resistors `R5=22Ω` and `R6=22Ω`
* onboard radio module exposes `UTX`, `URX`, `RESET`, `SWDIO`, `SWCLK`, and antenna connection through `ANT_IPEX`
* the antenna feed includes `R4=0Ω` to the IPEX antenna path

### Host wiring rule

Because the external connector labels are module-centric, connect MCU TX to module RX and MCU RX to module TX.

---

## 4. Device Addressing and Identification

The modem exposes identification commands rather than memory-mapped ID registers.

### Identification commands

| Item          | Command    | Example response         |
| ------------- | ---------- | ------------------------ |
| Manufacturer  | `AT+CGMI?` | `+CGMI=ASR`              |
| Model         | `AT+CGMM?` | `+CGMM=6501`             |
| Revision      | `AT+CGMR?` | `+CGMR=v4.0`             |
| Serial number | `AT+CGSN?` | `+CGSN=0539349E00032523` |

These commands and examples are documented in the AT reference .

### LoRaWAN identity/addressing fields

#### OTAA

* DevEUI: 16 hex characters = 8 bytes
* AppEUI: 16 hex characters = 8 bytes
* AppKey: 32 hex characters = 16 bytes 

#### ABP

* DevAddr: 8 hex characters = 4 bytes
* AppSKey: 32 hex characters = 16 bytes
* NwkSKey: 32 hex characters = 16 bytes 

### Discovery procedure

Recommended discovery:

1. open UART at `115200 8N1`
2. issue `AT+CGMI?`
3. issue `AT+CGMM?`
4. issue `AT+CGMR?`
5. optionally issue `AT+CGSN?`

A valid ASR6501 response is manufacturer `ASR`, model `6501`, and a revision string such as `v4.0` .

---

## 5. Communication Protocol

## 5.1 Host-to-device protocol model

The module uses line-oriented ASCII AT commands terminated by carriage return (`\r`, `0x0D`). Responses are line-oriented and may contain:

* parameter response lines beginning with `+<CMD>:`
* status lines such as `OK`
* asynchronous result lines such as `+CJOIN:OK`, `OK+SEND:03`, `OK+RECV:...`, `ERR+SEND:...` 

### Important protocol design rule

The host driver must support multi-line and asynchronous responses after a successful command submission. `AT+CJOIN=...` may return `OK` first and then later `+CJOIN:OK` or `+CJOIN:FAIL`. `AT+DTRX=...` may return several result lines for one transmit operation .

## 5.2 Example transactions

### Read manufacturer

Request:

```text
AT+CGMI?\r
```

Response:

```text
\r\n+CGMI=ASR\r\n
\r\nOK\r\n
```

### Start OTAA join with auto-join

```text
AT+CJOIN=1,1,10,8\r
```

Possible response:

```text
OK
+CJOIN:OK
```

or

```text
OK
+CJOIN:FAIL
```



### Send confirmed payload

```text
AT+DTRX=1,2,10,0123456789\r
```

Example response:

```text
OK+SEND:03
OK+SENT:01
OK+RECV:02,01,00
```



---

## 6. Register Map

This device does **not** expose a documented host-visible memory/register map over UART in the provided interface definition. The host-visible programming model is command-based only. All control is performed through AT commands; no register addresses are defined in the provided material .

---

## 7. Register Bitfields

Not applicable for the documented host interface. There are no host-visible register bitfields in the provided AT command definition.

---

## 8. Reserved Bit Handling

### For AT-command interface

No host-visible register reserved bits are defined.

### For structured fields returned by commands

Where response bitfields are defined, undocumented or reserved bits must be treated as follows:

* `OK+RECV:TYPE,...`:

  * `Bit4~Bit7`: reserved, default `0`
  * host code must mask only documented bits and ignore reserved bits on read 

---

## 9. Commands or Opcodes

## 9.1 Full command table

| Command                                                                         | Parameters                             | Description                                                            |
| ------------------------------------------------------------------------------- | -------------------------------------- | ---------------------------------------------------------------------- |
| `AT+CGMI?`                                                                      | none                                   | Read manufacturer identification                                       |
| `AT+CGMM?`                                                                      | none                                   | Read model identification                                              |
| `AT+CGMR?`                                                                      | none                                   | Read revision identification                                           |
| `AT+CGSN?`                                                                      | none                                   | Read product serial number                                             |
| `AT+CGBR=<baud>` / `?`                                                          | baud                                   | Set/read UART baud rate                                                |
| `AT+CJOINMODE=<mode>` / `?` / `=?`                                              | `0` OTAA, `1` ABP                      | Set/read join mode                                                     |
| `AT+CDEVEUI=<value>` / `?` / `=?`                                               | 16 hex chars                           | Set/read DevEUI                                                        |
| `AT+CAPPEUI=<value>` / `?` / `=?`                                               | 16 hex chars                           | Set/read AppEUI                                                        |
| `AT+CAPPKEY=<value>` / `?` / `=?`                                               | 32 hex chars                           | Set/read AppKey                                                        |
| `AT+CDEVADDR=<value>` / `?` / `=?`                                              | 8 hex chars                            | Set/read DevAddr                                                       |
| `AT+CAPPSKEY=<value>` / `?` / `=?`                                              | 32 hex chars                           | Set/read AppSKey                                                       |
| `AT+CNWKSKEY=<value>` / `?` / `=?`                                              | 32 hex chars                           | Set/read NwkSKey                                                       |
| `AT+CFREQBANDMASK=<mask>` / `?` / `=?`                                          | 4 hex chars, 16-bit group mask         | Set/read frequency band mask                                           |
| `AT+CULDLMODE=<mode>` / `?` / `=?`                                              | `1` same-freq, `2` different-freq      | Set/read UL/DL mode                                                    |
| `AT+CWORKMODE=<mode>` / `?` / `=?`                                              | only `2` supported                     | Set/read work mode                                                     |
| `AT+CCLASS=<class>[,...]` / `?` / `=?`                                          | class and optional class B params      | Set/read class                                                         |
| `AT+CBL?` / `=?`                                                                | none                                   | Read battery level                                                     |
| `AT+CSTATUS?` / `=?`                                                            | none                                   | Read device status                                                     |
| `AT+CJOIN=<p1>[,p2,p3,p4]` / `?` / `=?`                                         | join control, auto-join, period, retry | Start/stop join                                                        |
| `AT+DTRX=[confirm],[nbtrials],<Length>,<Payload>` / `=?`                        | send data                              | Send uplink / receive immediate result                                 |
| `AT+DRX?` / `=?`                                                                | none                                   | Read RX buffer and clear it                                            |
| `AT+CCONFIRM=<value>` / `?` / `=?`                                              | `0` unconfirmed, `1` confirmed         | Set/read uplink type                                                   |
| `AT+CAPPPORT=<value>` / `?` / `=?`                                              | `1..223` decimal (default `10`; `0` reserved for MAC) | Set/read application port                                |
| `AT+CDATARATE=<value>` / `?` / `=?`                                             | `0..5` (default `3`)                   | Set/read data rate                                                     |
| `AT+CRSSI <FREQBANDIDX>?` / `=?`                                                | band index                             | Read RSSI for one frequency group                                      |
| `AT+CNBTRIALS=<MType>,<value>` / `?` / `=?`                                     | type, retry count `1..15`              | Set/read send times                                                    |
| `AT+CRM=<reportMode>[,reportInterval]` / `?` / `=?`                             | report mode and interval               | Set/read upload mode                                                   |
| `AT+CTXP=<value>` / `?` / `=?`                                                  | power index                            | Set/read TX power                                                      |
| `AT+CLINKCHECK=<value>` / `=?`                                                  | `0,1,2`                                | Link check control                                                     |
| `AT+CADR=<value>` / `?` / `=?`                                                  | `0` disable, `1` enable                | Set ADR                                                                |
| `AT+CRXP=<RX1DRoffest>,<RX2DataRate>,<RX2Frequency>` / `?` / `=?`               | RX window params                       | Set/read RX-window params                                              |
| `AT+CFREQLIST=<ULDL>,<method>,<number>,<freqlist>` / `?` / `=?`                 | frequency table config                 | Optional frequency-table config; document says not currently supported |
| `AT+CRX1DELAY=<Delay>` / `?` / `=?`                                             | seconds                                | Set/read RX1 delay                                                     |
| `AT+CSAVE` / `=?`                                                               | none                                   | Save MAC config to EEPROM/FLASH                                        |
| `AT+CRESTORE` / `=?`                                                            | none                                   | Restore default MAC config                                             |
| `AT+CPINGSLOTINFOREQ=<periodicity>` / `?` / `=?`                                | periodicity                            | Ping slot info request (Class B only)                                  |
| `AT+CADDMUTICAST=<DevAddr>,<AppSKey>,<NwkSKey>,[Periodicity],[Datarate]` / `=?` | multicast entry                        | Add multicast address                                                  |
| `AT+CDELMUTICAST=<DevAddr>` / `=?`                                              | DevAddr                                | Delete multicast address                                               |
| `AT+CNUMMUTICAST?` / `=?`                                                       | none                                   | Read multicast count                                                   |
| `AT+IREBOOT=<mode>` / `=?`                                                      | `0,1,7`                                | Reboot / bootloader                                                    |
| `AT+ILOGLVL=<level>` / `?` / `=?`                                               | `0..5`                                 | Set/read log level                                                     |
| `AT+CKEYSPROTECT=<key>` / `?` / `=?`                                            | 32 hex chars                           | Encrypt device triple-tuple                                            |
| `AT+CLPM=<mode>` / `=?`                                                         | `1` documented                         | Enable low-power mode                                                  |
| `AT+CSLEEP=<sleep_mode>` / `=?`                                                 | `0,1,2`                                | Deep sleep test                                                        |
| `AT+CMCU=<mcu_mode>` / `=?`                                                     | `0,1,2,3`                              | MCU low-power test                                                     |
| `AT+CSTDBY=<standby_mode>` / `=?`                                               | `0,1`                                  | SX1262 standby + MCU deep sleep                                        |
| `AT+CRX=<freq>,<data_rate>` / `=?`                                              | frequency, DR0..DR5                    | Continuous RX test                                                     |
| `AT+CTX=<freq>,<data_rate>,<pwr>` / `=?`                                        | frequency, DR, power                   | Repeating TX test                                                      |
| `AT+CTXCW=<freq>,<pwr>[,<opt>]` / `=?`                                          | frequency, power, PA option            | Continuous-wave TX test                                                |

Derived from command summary and detailed per-command sections .

`CFREQLIST` is detailed in section 5.2.32 but absent from the summary tables.
`DTRX`, `CLINKCHECK`, `CSAVE`, `CRESTORE`, `CLPM`, `CSLEEP`, `CMCU`,
`CSTDBY`, `CRX`, `CTX`, `CTXCW`, `CADDMUTICAST`, `CDELMUTICAST`, and
`IREBOOT` have no `?` inquire form. `CGMI`, `CGMM`, `CGMR`, `CGSN`, and
`CGBR` have no `=?` test form.

## 9.2 Per-command details

* `CFREQBANDMASK`: each bit enables one 8-channel group. Channels 0-7 are
  `0001` and channels 8-15 are `0002`. Set it before join.
* `CCLASS`: for `class=1,branch=0`, `para1` is the ping-slot periodicity
  `0..7` (see 11.3). For `class=1,branch=1`, `para1` is the beacon frequency
  in Hz, `para2` the beacon data rate, `para3` the ping-slot frequency in Hz,
  and `para4` the ping-slot data rate. Set the class before join.
* `CJOIN` `p1`: `0` stops join; `1` starts a new join, clearing the join
  parameters on modules with warm boot enabled. `p2` enables auto-join
  (`0`/`1`, factory `1`). `AT+CJOIN?` returns the current `p1..p4`.
* `CRSSI`: syntax is `AT+CRSSI <idx>?` with a space. The index starts at `0`.
  The reply is `+CRSSI:` followed by one `<channel>:<rssi>` line for each of
  the group's 8 channels, then `OK`.
* `DRX`: with an empty buffer, the example returns only `OK`.
* `CADDMUTICAST`: add entries before join. The example uses lowercase hex.
* `CFREQLIST`: `ULDL` is `1` uplink or `2` downlink (downlink only for
  different-frequency nodes). `method` is `1` (start frequency plus channel
  count) or `2` (explicit list). `number` is `1..16` and frequencies are in Hz.
  The v4.0 PDF says the command is not supported.
* `CRM`: mainly for testing.
* `CKEYSPROTECT?` returns `+CKEYSPROTECT:<protected>`.
* Set before join: `CFREQBANDMASK`, `CULDLMODE`, `CWORKMODE`, `CCLASS`,
  `CADDMUTICAST`. Set before sending: `CCONFIRM`, `CAPPPORT`, `CDATARATE`,
  `CNBTRIALS`, `CRM`, `CTXP`, `CLINKCHECK`, `CADR`, `CRXP`, `CRX1DELAY`,
  `CSAVE`.

## 9.3 Source errata (v4.0)

* 5.2.4: the `CGSN` format row shows `+CGMR=<sn>`; the example shows `+CGSN=`.
* 5.2.25: the `CRSSI` format lists up to `15:<Channel 8 rssi>`, but the
  text and example return channels `0..7`.
* 5.2.27: the `CRM` inquire and execute rows show `+CTXP` / `AT+CTXP`. The
  command is `CRM` (example `AT+CRM=1,10`).
* 5.2.29: the `CLINKCHECK` example uses full-width commas (U+FF0C) and a
  space after the colon. The modem's actual bytes are unverified.
* 5.2.34: `CSAVE` refers to `AT+RESET`, which is not defined (see 13.2).
* 5.2.43: `CLPM` defines only mode `1`, but its notice mentions `AT+CLPM=0`.
* 5.2.46: the `CSTDBY` test reply shows `+CRXC = <0, 1>`.

---

## 10. Data Formats

## 10.1 ASCII command encoding

All commands and parameter strings are ASCII text terminated with `\r` (`0x0D`) .

## 10.2 Hex string fields

* DevEUI: 16 hex chars = 8 bytes
* AppEUI: 16 hex chars = 8 bytes
* AppKey: 32 hex chars = 16 bytes
* DevAddr: 8 hex chars = 4 bytes
* AppSKey: 32 hex chars = 16 bytes
* NwkSKey: 32 hex chars = 16 bytes
* DTRX payload: hexadecimal string; two characters represent one digit/byte according to the document wording 

## 10.3 Uplink send format

`AT+DTRX=[confirm],[nbtrials],<Length>,<Payload>`

* `confirm`: optional override for this send
* `nbtrials`: optional override for this send
* `Length`: number of hexadecimal characters, twice the payload byte count
* `Payload`: hex string
* `0` length represents empty packet 

### Practical driver rule

Treat payload as hex-encoded bytes and validate:

* even number of hex characters
* encoded character count equals `<Length>`

PDF section 5.2.20 gives `DTRX=1,2,10,0123456789`: five encoded bytes,
ten characters. The byte-buffer API must send `2 * byte_count` as Length.
Do not conflate the byte-sized receive LEN field with transmit Length.
The same example reports `OK+SEND:03` for a 5-byte payload, so do not
validate `TX_LEN` against the sent length.

## 10.4 Downlink indication format

`OK+RECV:TYPE,PORT,LEN,DATA`

Where:

* `TYPE`: 1 byte bitfield

  * Bit0: `0` unconfirm, `1` confirm
  * Bit1: `0` non-ACK, `1` ACK
  * Bit2: `0` non-carry, `1` carry ack of LINK command
  * Bit3: `0` non-carry, `1` carry ack of TIME command; when `1`, time sync success
  * Bit4..Bit7: reserved, default `0`
* `PORT`: 1 byte transport port
* `LEN`: 1 byte downlink data length
* `DATA`: n-byte data, absent when `LEN=0` 

The PDF does not state the radix of `TYPE`, `PORT`, and `LEN`. M5Stack's
reference library assumes fixed two-character fields, which implies hex
bytes. The driver parses them as hex and accepts `LEN` in either radix when it
matches the DATA length.

## 10.5 RSSI format

`AT+CRSSI <FREQBANDIDX>?` returns one RSSI value per channel in a frequency group. Example values are signed integers such as `-157` .

## 10.6 Data rate enumeration

`AT+CDATARATE` values:

* `0` = `SF12, BW125`
* `1` = `SF11, BW125`
* `2` = `SF10, BW125`
* `3` = `SF9, BW125`
* `4` = `SF8, BW125`
* `5` = `SF7, BW125` 

Factory default data rate is `3`. After ADR is enabled, a manual `CDATARATE` setting loses effect.

### Send-times constraint

`AT+CNBTRIALS=<MType>,<value>` retry count range is `1..15` (`MType`: `0` unconfirmed, `1` confirmed). This is distinct from the `AT+CJOIN` join retry range of `1..256`. 

## 10.7 TX power data formats

### LoRaWAN TX power command

`AT+CTXP=<value>` uses a product-dependent power index. The only explicit mapping in the AT reference is for CN470A:

* `0` = `17 dBm`
* `1` = `15 dBm`
* `2` = `13 dBm`
* `3` = `11 dBm`
* `4` = `9 dBm`
* `5` = `7 dBm`
* `6` = `5 dBm`
* `7` = `3 dBm` 

### Radio test commands

For `AT+CTX` and `AT+CTXCW`, `pwr` is the SX1262 TX power with range `0..22` .

---

## 11. Timing Requirements

## 11.1 UART timing

* UART baud default/documented console rate: `115200`
* frame format: `8N1` 

## 11.2 Join timing

`AT+CJOIN=<Para1>,<Para2>,<Para3>,<Para4>`

* join period (`ParaTag3`) range: `7..255` seconds
* default join period: `8` seconds
* max retry count (`ParaTag4`) range: `1..256` 

## 11.3 Class B ping slot periodicity

If `class=1` and `branch=0`, `para1` range is `0..7` and ping-slot period is:

`0.96 * 2^periodicity seconds` 

## 11.4 Report mode minimum intervals

For `AT+CRM=1,<reportInterval>`, minimum interval by data rate:

| Data rate | LV1 | LV2 |
| --------- | --: | --: |
| DR0       | 150 | 300 |
| DR1       |  75 | 150 |
| DR2       |  35 |  70 |
| DR3       |  15 |  30 |
| DR4       |  10 |  20 |
| DR5       |   5 |  10 |

Units are seconds .

## 11.5 RX1 delay

`AT+CRX1DELAY=<Delay>`

* Delay unit: seconds
* meaning: how many seconds to open RX1 window after TX done 

## 11.6 Sleep/wakeup timings

* `AT+CSLEEP=0` enters deep sleep and wakes by timer `10 s` later; example output: `deep sleep 10000 ms!=0` and `+CSLEEP` 
* `AT+CMCU=3` enters deep sleep every `15 s` 
* `AT+CTX` transmits at `1 s` interval in loop mode 

---

## 12. Operating Modes

## 12.1 Join modes

* `0` = OTAA
* `1` = ABP
* default = OTAA 

## 12.2 Work mode

* only documented supported value: `2` = Normal Work Mode
* default = normal work mode
* currently only normal work mode is supported 

## 12.3 LoRaWAN classes

* `0` = Class A
* `1` = Class B
* `2` = Class C
* default = Class A 

## 12.4 Message confirmation mode

* `0` = unconfirmed uplink
* `1` = confirmed uplink 

## 12.5 ADR

* `0` = disable ADR
* `1` = enable ADR
* default = enabled 

## 12.6 UL/DL mode

* `1` = same frequency mode
* `2` = different frequency mode 

## 12.7 Power modes

* `AT+CLPM=1` = enter low-power mode
* `AT+CSLEEP=0/1/2` = deep sleep test with different wake sources
* `AT+CMCU=0/1/2/3` = MCU/radio power test modes
* `AT+CSTDBY=0/1` = SX1262 standby mode select with MCU deep sleep 

## 12.8 Radio test modes

* `AT+CRX=<freq>,<data_rate>` continuous RX
* `AT+CTX=<freq>,<data_rate>,<pwr>` periodic TX loop (1 s interval)
* `AT+CTXCW=<freq>,<pwr>[,<opt>]` continuous-wave TX
* radio-test frequency range for all three: `150000000`..`960000000` Hz
* `data_rate` for `CRX`/`CTX`: `DR0`..`DR5` (SF12..SF7)
* `pwr` for `CTX`/`CTXCW`: SX1262 TX power `0..22`
* `CTXCW` `opt` is the SX1262 PA-optimal setting, range `0..3`, default `0`:
  * `0` = `[0x04,0x07,0x00,0x01]`
  * `1` = `[0x03,0x05,0x00,0x01]`
  * `2` = `[0x02,0x03,0x00,0x01]`
  * `3` = `[0x02,0x02,0x00,0x01]`
* all are effectively terminal test modes and require reboot before returning to normal operation because the system enters a dead loop 

---

## 13. Reset Behavior

## 13.1 Reboot command

`AT+IREBOOT=<mode>`

* `0` = reboot immediately
* `1` = reboot after current frame transmission completes
* `7` = reboot and enter bootloader 

After the module responds `OK`, it reboots and will not receive other AT commands before reboot completes .

## 13.2 Configuration persistence and reset

`AT+CSAVE` stores MAC configuration to EEPROM/FLASH. After `AT+RESET`, the module uses the new MAC configuration parameters to initialize the network. The AT reference explicitly mentions `AT+RESET`, but the listed reboot command in the same document is `AT+IREBOOT`; generated code should treat this as documentation inconsistency and prefer `AT+IREBOOT` for reboot control while recognizing that persisted settings are intended to survive reboot .

## 13.3 Factory restore

`AT+CRESTORE` restores MAC default configuration parameters into EEPROM/FLASH .

---

## 14. Status and Diagnostics

## 14.1 Device status

`AT+CSTATUS?` returns one of:

* `00` no data operation
* `01` data in sending
* `02` data sent but failed
* `03` data sent and success
* `04` join success, only in first join procedure
* `05` join fail, only in first join procedure
* `06` network may be abnormal, result from link check
* `07` data sent successfully but no downlink
* `08` data sent successfully and downlink exists 

## 14.2 Send diagnostics

For `AT+DTRX`:

* `OK+SEND:TX_LEN` = send accepted/success indication, 1-byte TX length
* `OK+SENT:TX_CNT` = send success, 1-byte transmit count
* `ERR+SEND:ERR_NUM` = send failure reason

  * `0` not joined to network successfully
  * `1` communication path busy, send failed
  * `2` data length exceeded allowable length, only MAC command sent
* `ERR+SENT:TX_CNT` = send failed because retries exceeded maximum count 

## 14.3 Link diagnostics

`AT+CLINKCHECK=1` later returns:

`+CLINKCHECK:Y0,Y1,Y2,Y3,Y4`

* `Y0` result: `0` success, non-zero fail
* `Y1` DemodMargin
* `Y2` NbGateways
* `Y3` RSSI of downlink for the command
* `Y4` SNR of downlink for the command 

## 14.4 Logging

`AT+ILOGLVL=<level>`

* `0` disable log information
* `1..5` enable logging with increasing verbosity 

---

## 15. Interrupts

No dedicated host interrupt pin behavior is documented in the AT-command reference. Host integration should treat the device as a polled/asynchronous UART peripheral.

Relevant hardware control pins visible on the schematic include `RESET`, `SWDIO`, and `SWCLK`, but no documented host interrupt signal is provided in the user-supplied material.

---

## 16. Operational Sequences

## 16.1 OTAA provisioning sequence

1. open UART at `115200 8N1`
2. verify modem identity using `CGMI`, `CGMM`, `CGMR`
3. set join mode: `AT+CJOINMODE=0`
4. set `DevEUI`
5. set `AppEUI`
6. set `AppKey`
7. set frequency band mask: `AT+CFREQBANDMASK=<mask>`
8. optionally set UL/DL mode, work mode, class, ADR, app port, data rate, confirm mode, retries, RX params
9. optionally `AT+CSAVE`
10. start join: `AT+CJOIN=1,1,<period>,<max_retry>`
11. wait for asynchronous `+CJOIN:OK` or `+CJOIN:FAIL` 

## 16.2 ABP provisioning sequence

1. `AT+CJOINMODE=1`
2. set `DevAddr`
3. set `AppSKey`
4. set `NwkSKey`
5. set frequency-related parameters
6. optionally save
7. proceed to send data without OTAA authentication flow 

## 16.3 Send data sequence

1. ensure network joined
2. optionally configure:

   * `AT+CCONFIRM`
   * `AT+CAPPPORT`
   * `AT+CDATARATE`
   * `AT+CNBTRIALS`
   * `AT+CADR`
3. send using `AT+DTRX=[confirm],[nbtrials],<Length>,<Payload>`
4. parse `OK+SEND`
5. parse `OK+SENT` or `ERR+SENT`
6. parse optional `OK+RECV` downlink 

## 16.4 Read buffered RX sequence

1. issue `AT+DRX?`
2. parse `+DRX:<Length>,<Payload>`
3. note that RX buffer is cleared after read 

## 16.5 Save configuration sequence

1. perform desired configuration commands
2. issue `AT+CSAVE`
3. reboot module if needed to reinitialize with saved settings 

## 16.6 Restore defaults sequence

1. issue `AT+CRESTORE`
2. reboot module to restart with restored defaults
3. reprovision as needed 

---

## 17. Driver Initialization Sequence

Recommended driver startup:

1. power module from `5V` supply through board connector
2. wait for UART availability after boot
3. open UART at `115200 8N1`
4. flush incoming garbage/log lines
5. issue:

   * `AT+CGMI?`
   * `AT+CGMM?`
   * `AT+CGMR?`
6. verify returned model/manufacturer match expected ASR6501
7. optionally set log level to known state: `AT+ILOGLVL=0` or configured verbosity
8. query or apply join parameters and runtime defaults
9. if persistent config is used, either trust existing values or fully rewrite all required parameters and `AT+CSAVE`
10. if a reboot is required for a known state, use `AT+IREBOOT=0` and reconnect after boot

---

## 18. Runtime Operation Sequences

## 18.1 Periodic telemetry

1. verify join state
2. set port if needed
3. choose confirmation mode
4. encode payload to hex
5. transmit via `AT+DTRX`
6. update state based on `OK+SEND`, `OK+SENT`, `OK+RECV`, or error lines
7. optionally poll `AT+CSTATUS?`

## 18.2 Link quality validation

1. issue `AT+CLINKCHECK=1`
2. wait for asynchronous `+CLINKCHECK:Y0,Y1,Y2,Y3,Y4`
3. if `Y0 != 0`, flag degraded network condition
4. optionally query `AT+CSTATUS?` and `AT+CRSSI <idx>?`

## 18.3 Changing data rate

1. if ADR enabled, know that manual `CDATARATE` setting loses effect
2. disable ADR if fixed manual rate is required: `AT+CADR=0`
3. set `AT+CDATARATE=<0..5>`
4. confirm with `AT+CDATARATE?` 

## 18.4 Enter low-power mode

1. ensure no critical AT transaction is outstanding
2. issue `AT+CLPM=1` or a test sleep command
3. on wake, reestablish UART parser sync
4. if sleep mode is destructive to session handling in your application, re-query status or rejoin as policy requires

---

## 19. Driver State Model

The driver should track at minimum:

* UART connection state
* modem identity strings
* join mode: OTAA or ABP
* join state: idle, joining, joined, join failed
* LoRaWAN class
* confirmation mode
* app port
* data rate
* ADR enabled state
* retry settings
* RX1 delay
* RX window settings
* saved vs unsaved configuration dirty flag
* pending TX operation
* last status code from `CSTATUS`
* receive buffer / last downlink
* log level
* low-power state
* whether modem is in dead-loop test mode and requires reboot

---

## 20. Required Safety and Correctness Rules for Code Generation

1. Always terminate commands with carriage return `\r`.
2. Always configure UART as `115200 8N1` unless intentionally changed with `CGBR`.
3. Parse multi-line and delayed asynchronous responses; do not assume one command maps to one line.
4. Do not send payload data before successful join when using LoRaWAN network operation.
5. Set OTAA credentials only in OTAA mode and ABP credentials only in ABP mode.
6. Set `CFREQBANDMASK`, class, work mode, and other join-sensitive parameters before joining when required by command notices.
7. Treat `AT+CFREQLIST` as unsupported in this implementation path; prefer `AT+CFREQBANDMASK`.
8. When ADR is enabled, manual `CDATARATE` settings must not be assumed effective.
9. Validate hex-string field lengths exactly:

   * DevEUI/AppEUI = 16
   * AppKey/AppSKey/NwkSKey = 32
   * DevAddr = 8
10. Validate that payload hex strings contain an even number of characters.
11. Treat `OK+RECV TYPE` reserved bits `4..7` as reserved and ignore them.
12. After `AT+DRX?`, assume RX buffer has been cleared.
13. After `AT+IREBOOT`, do not send further commands until the module has rebooted and UART is available again.
14. Entering `CRX`, `CTX`, or `CTXCW` test mode places the system in a dead loop; require reboot before normal operation.
15. `CKEYSPROTECT` is effectively irreversible for normal host use; do not invoke automatically.
16. Before low-power entry, ensure host parser is ready for wakeup behavior and possible UART corruption at high speed.
17. For `CLPM`, respect the documented warning that when transmit speed is `> 40kbps`, the UART start byte may be received incorrectly; the document recommends using wakeup data `000000000D0A` in hex for wakeup operation .
18. Save configuration explicitly with `CSAVE`; do not assume prior set commands are persistent across reboot.
19. When using reboot mode `1`, allow in-flight frame transmission to complete.
20. Surface all `+CME ERROR:<err>` values to upper layers; the PDF defers numeric meanings to 3GPP and does not list them.

---

## 21. Canonical Constants

```text
ASR6501_UART_BAUD_DEFAULT = 115200
ASR6501_UART_DATA_BITS = 8
ASR6501_UART_STOP_BITS = 1
ASR6501_UART_PARITY = NONE

ASR6501_JOIN_MODE_OTAA = 0
ASR6501_JOIN_MODE_ABP  = 1

ASR6501_WORK_MODE_NORMAL = 2

ASR6501_CLASS_A = 0
ASR6501_CLASS_B = 1
ASR6501_CLASS_C = 2

ASR6501_CONFIRM_UNCONFIRMED = 0
ASR6501_CONFIRM_CONFIRMED   = 1

ASR6501_ULDL_SAME_FREQ      = 1
ASR6501_ULDL_DIFFERENT_FREQ = 2

ASR6501_ADR_DISABLE = 0
ASR6501_ADR_ENABLE  = 1

ASR6501_REBOOT_IMMEDIATE      = 0
ASR6501_REBOOT_AFTER_TX       = 1
ASR6501_REBOOT_ENTER_BOOTLOADER = 7

ASR6501_LOG_DISABLE = 0
ASR6501_LOG_1 = 1
ASR6501_LOG_2 = 2
ASR6501_LOG_3 = 3
ASR6501_LOG_4 = 4
ASR6501_LOG_5 = 5

ASR6501_DR0 = 0  // SF12 BW125
ASR6501_DR1 = 1  // SF11 BW125
ASR6501_DR2 = 2  // SF10 BW125
ASR6501_DR3 = 3  // SF9 BW125
ASR6501_DR4 = 4  // SF8 BW125
ASR6501_DR5 = 5  // SF7 BW125
```

### Status constants

```text
ASR6501_STATUS_IDLE                = 0x00
ASR6501_STATUS_SENDING             = 0x01
ASR6501_STATUS_SEND_FAILED         = 0x02
ASR6501_STATUS_SEND_SUCCESS        = 0x03
ASR6501_STATUS_JOIN_SUCCESS        = 0x04
ASR6501_STATUS_JOIN_FAIL           = 0x05
ASR6501_STATUS_NETWORK_ABNORMAL    = 0x06
ASR6501_STATUS_SEND_OK_NO_DL       = 0x07
ASR6501_STATUS_SEND_OK_WITH_DL     = 0x08
```

### DTRX error constants

```text
ASR6501_SEND_ERR_NOT_JOINED      = 0
ASR6501_SEND_ERR_PATH_BUSY       = 1
ASR6501_SEND_ERR_LENGTH_EXCEEDED = 2
```

---

## 22. Bitfield Constants

### Downlink TYPE bitfield from `OK+RECV:TYPE,PORT,LEN,DATA`

```text
ASR6501_RECV_TYPE_CONFIRM_MASK   = 0x01
ASR6501_RECV_TYPE_CONFIRM_SHIFT  = 0

ASR6501_RECV_TYPE_ACK_MASK       = 0x02
ASR6501_RECV_TYPE_ACK_SHIFT      = 1

ASR6501_RECV_TYPE_LINK_ACK_MASK  = 0x04
ASR6501_RECV_TYPE_LINK_ACK_SHIFT = 2

ASR6501_RECV_TYPE_TIME_ACK_MASK  = 0x08
ASR6501_RECV_TYPE_TIME_ACK_SHIFT = 3

ASR6501_RECV_TYPE_RESERVED_MASK  = 0xF0
ASR6501_RECV_TYPE_RESERVED_SHIFT = 4
```

---

## 23. Enumerations

## 23.1 Join mode

```text
enum asr6501_join_mode {
  ASR6501_JOIN_OTAA = 0,
  ASR6501_JOIN_ABP  = 1
};
```

## 23.2 Device class

```text
enum asr6501_class {
  ASR6501_CLASS_A = 0,
  ASR6501_CLASS_B = 1,
  ASR6501_CLASS_C = 2
};
```

## 23.3 Confirmation mode

```text
enum asr6501_confirm_mode {
  ASR6501_UNCONFIRMED = 0,
  ASR6501_CONFIRMED   = 1
};
```

## 23.4 ADR mode

```text
enum asr6501_adr_mode {
  ASR6501_ADR_OFF = 0,
  ASR6501_ADR_ON  = 1
};
```

## 23.5 Reboot mode

```text
enum asr6501_reboot_mode {
  ASR6501_REBOOT_NOW        = 0,
  ASR6501_REBOOT_AFTER_TX   = 1,
  ASR6501_REBOOT_BOOTLOADER = 7
};
```

## 23.6 Sleep/test modes

```text
enum asr6501_csleep_mode {
  ASR6501_CSLEEP_TIMER_10S = 0,
  ASR6501_CSLEEP_SET_B     = 1,
  ASR6501_CSLEEP_UART      = 2
};

enum asr6501_cmcu_mode {
  ASR6501_CMCU_PD_SX1262      = 0,
  ASR6501_CMCU_MCU_ACTIVE     = 1,
  ASR6501_CMCU_DEEPSLEEP_SETB = 2,
  ASR6501_CMCU_DEEPSLEEP_15S  = 3
};

enum asr6501_cstdby_mode {
  ASR6501_STDBY_RC   = 0,
  ASR6501_STDBY_XOSC = 1
};
```

---

## 24. Minimal Functional Feature Set

A minimally correct driver implementation must provide:

1. UART transport at `115200 8N1`
2. AT command send/receive with CR termination
3. asynchronous multi-line response parsing
4. identity readout (`CGMI`, `CGMM`, `CGMR`)
5. OTAA configuration and join
6. ABP configuration support
7. uplink send using `DTRX`
8. downlink parsing from `OK+RECV`
9. status query using `CSTATUS`
10. configuration of confirmation mode, port, data rate, ADR, retries
11. reboot handling with reconnect
12. persistence support through `CSAVE` and `CRESTORE`

---

## 25. Final Implementation Intent

A generated driver should behave like a robust asynchronous UART modem driver, not like a register driver. It should maintain a clear internal state machine, validate parameters before sending them, serialize outstanding command transactions, parse delayed result indications, and preserve network correctness across joins, data transfers, save/restore, reboot, and low-power transitions.

Common implementation mistakes to avoid:

* treating the module like a synchronous one-line AT device
* assuming `OK` means the operation is fully complete
* sending data before join completion
* using manual data rate while ADR is enabled and expecting it to apply
* failing to clear or account for RX buffer behavior after `DRX?`
* automatically calling irreversible key protection
* entering radio test mode without planning a reboot path
* ignoring the difference between OTAA and ABP credential sets
* using unsupported `CFREQLIST` instead of `CFREQBANDMASK`
* mis-parsing connector RX/TX direction on the external HY2.0 port
