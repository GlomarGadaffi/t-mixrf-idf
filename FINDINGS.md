# T-MixRF: Findings & Brainstorm

Analysis of the LILYGO T-MixRF [H775] hardware, pure ESP-IDF implementation notes,
and integration ideas for the GlomarGadaffi RF stack.

**All frequency defaults in this repo target North American ISM bands (FCC Part 15):**
- CC1101: 315 MHz, 433 MHz, or **915 MHz** (902–928 MHz). **Not 868 MHz** — that's EU ISM.
- LR1121: **915 MHz** (902–928 MHz) or 2.4 GHz (2400–2483.5 MHz).
- nRF24L01: 2.4 GHz, channels 0–83 (2400–2483 MHz) — fine everywhere.
- `radio-cc` and `fsk-walkie-talkie` already default to 914.6/915 MHz — correct for NA.

---

## Hardware summary

| Component | Role | Interface | Key pins |
|---|---|---|---|
| ESP32-S3-WROOM-1-N16R8 | MCU | — | 16 MB flash, 8 MB OPI PSRAM (disabled — see below) |
| LR1121 | Dual-band LoRa (150–960 MHz + 2.4 GHz) | SPI (shared) | CS=33, DIO9=9, RST=37, BUSY=6, SW=7 |
| CC1101 | Sub-GHz FSK/OOK 315/433/868/915 MHz | SPI (shared) | CS=11, GDO0=14, GDO2=12, SW0=13, SW1=10 |
| nRF24L01+ | 2.4 GHz, up to 2 Mbps | SPI (shared) | CS=34, CE=4, IRQ=5 |
| ST25R3916 | NFC ISO14443A/B, ISO15693, NFC-IP | SPI (shared) | BSS=36, EN=35 |

**Shared SPI bus:** SCK=17, MOSI=15, MISO=16 (SPI2_HOST)

### Critical: PSRAM must be disabled

GPIO 33–37 are the OPI PSRAM interface inside the WROOM-1-N16R8 module.
Those same pads are wired to LR1121-CS (33), nRF24-CS (34), NFC-EN (35),
NFC-CS (36), and LR1121-RST (37). `CONFIG_SPIRAM=n` in sdkconfig.defaults
is non-negotiable — otherwise the PSRAM controller claims those GPIOs at
boot and none of the radio devices will respond.

---

## Implementation notes

### LR1121 two-phase SPI

The LR1121 uses a BUSY-gated two-phase protocol:

```
Phase A: CS↓ | opcode[2B] + params[NB] | CS↑  →  wait BUSY=LOW
Phase B: CS↓ | NOP[1B] | response[MB]  | CS↑   (read commands only)
```

TCXO: 3.3 V = byte value `0x07` (not `0x06`, which is 3.0 V).
Stabilisation delay: 164 ticks × 30.5 µs ≈ 5 ms.

Opcodes are verified against RadioLib `LR11x0_commands.h`:
- System group: `0x01xx`
- Radio group:  `0x02xx`

### CC1101 SPI

Standard Mode 0. Header byte: `R/W(7) | BURST(6) | ADDR[5:0]`.
Status registers require the BURST bit set even for single reads (§10.4).
SRES strobe is sufficient for reset after `spi_bus_add_device`; the
power-on CS toggle sequence is only needed for cold-boot recovery.

RSSI conversion: `dBm = (raw ≥ 128) ? raw/2 − 74 : raw/2 + 128 − 74`

Frequency presets (26 MHz crystal):
- 315 MHz: `{0x0C, 0x1D, 0x89}`
- 433 MHz: `{0x10, 0xA7, 0x62}`
- 868 MHz: `{0x21, 0x62, 0x76}`
- 915 MHz: `{0x23, 0x31, 0x3B}`

Band switch: SW1=GPIO10, SW0=GPIO13
- 315: SW1=1 SW0=0
- 433: SW1=1 SW0=1
- 868/915: SW1=0 SW0=1

### nRF24L01+

Mode 0. Address bytes are sent LSByte first per Nordic spec.
Dynamic payload length: set `EN_DPL` in `FEATURE` and `DYNPD`.
Max retransmit → `MAX_RT` flag set in STATUS; must flush TX FIFO after.

### ST25R3916

SPI Mode 1 (CPOL=0 CPHA=1). EN pin (GPIO35) must be driven HIGH before
SPI traffic — it powers the analog front-end.
IC_IDENT register (0x3F) returns `0x09` (ST25R3916) or `0x0B` (ST25R3916B).
For production NFC use integrate ST's RFAL library.

---

## Brainstorm: integration with the GlomarGadaffi RF stack

### The gap CC1101 fills

The existing stack (radio-cc, fsk-walkie-talkie, deauth-detector) operates
on LoRa (SX1262/SX1276) and WiFi/BLE. Nothing covers legacy sub-GHz ISM.

CC1101 at 315/433/868/915 MHz unlocks:

- **Passive ISM intelligence** — TPMS sensors, keyfobs, weather stations, door
  sensors all broadcast OOK/FSK in this range. Feed into the `ask-the-logs`
  BigQuery pipeline for NL queries over ambient RF traffic ("how many cars
  were in the lot between 0800–1200?" via TPMS MAC tracking).
- **Sub-GHz replay/fuzzing** — complements `lora-rf-toolkit`'s analysis pipeline
  and `rfparty`'s BLE fingerprinting.
- **Codec2 voice at 433 MHz** — `fsk-walkie-talkie` uses SX1262 in raw FSK
  at 19.2 kbps. CC1101 handles FSK up to ~500 kbps; Codec2 3200 (3.2 kbps)
  fits easily and 433 MHz is often a cleaner regulatory fit than 914.6 MHz.
- **Cheap `deauth-detector` sensor nodes** — CC1101 + bare ESP32 BOM is lower
  than SX1262; the 17-byte `alert_t` packet fits CC1101's 64-byte FIFO with room.

### LR1121: what radio-cc actually needs

`radio-cc` runs three separate spikes (raw LoRa / MeshCore / Reticulum) each
on a dedicated ESP32 + SX1262. LR1121 collapses this:

- **Reticulum backbone** on LR1121 868 MHz (long range, sparse nodes)
- **MeshCore local cluster** on LR1121 2.4 GHz (short range, dense nodes, faster)
- Band-hop on congestion — if 2.4 GHz saturates (WiFi/BLE), fall back to 868.

The Orbic server WebSocket protocol doesn't change; the gateway just reports
`freq_mhz` for the band the packet arrived on.

`fsk-walkie-talkie`: LR1121 is a pin-compatible SPI upgrade from SX1262. Gain
dual-band (868 MHz long range / 2.4 GHz close range) and better TCXO stability
during TX heating, at no firmware-architecture cost.

### nRF24L01: low-latency last meter

- **PTT remote** for `fsk-walkie-talkie`: a small nRF24 node on the wrist sends
  PTT at <5 ms without tying up the LoRa radio.
- **Wireless `pocket-dial-handset`**: nRF24 at 250 kbps carries G.711 (64 kbps)
  with headroom and avoids WiFi association latency. T-MixRF = belt-clip RTP
  mixer; earpiece = dumb nRF24 endpoint.
- **`project-ashburn-cockpit` internal bus**: short-range vehicle sensor telemetry
  without BLE stack overhead.

### ST25R3916 NFC: field provisioning and auth

- **`pocket-dial` tap-to-register**: NDEF record `sip:ext@192.168.4.1` on a tag;
  tap configures the handset without a serial console. Replaces NVS provisioning
  flow for field deployment.
- **`radio-cc` operator auth**: default mode = RX only; NFC card tap enables TX.
  Physical proof-of-presence instead of web UI password.
- **`project-ashburn-cockpit` dashboard lock**: MIFARE card = vehicle unlock +
  RF dashboard enable.

### The universal gateway

```
┌────────────────────────────────────────────────────────────┐
│  T-MixRF (ESP32-S3)  "universal gateway"                   │
│                                                             │
│  LR1121 @ 868 MHz   → Reticulum backbone                  │
│  LR1121 @ 2.4 GHz   → MeshCore local cluster              │
│  CC1101 @ 433 MHz   → ISM passive / sensor net            │
│  nRF24  @ 2.4 GHz   → PTT controller / handset link      │
│  NFC                → operator tap-to-enable TX            │
│  WiFi (ESP32-S3)    → WebSocket to Orbic C&C (7878)      │
└────────────────────────────────────────────────────────────┘
```

Single board, single firmware image, same JSON WebSocket protocol toward Orbic.

### Follow-on projects

| Project | Extends | What it does |
|---|---|---|
| `t-mixrf-subghz-scanner` | `lora-rf-toolkit`, `ask-the-logs` | CC1101 sweep + OOK/FSK demod → BigQuery |
| `t-mixrf-radio-bridge` | `radio-cc` | Multi-protocol L2 bridge (routing table in ESP32, Orbic = control plane) |
| `t-mixrf-nfc-provisioner` | `pocket-dial` | NFC tap-to-configure extensions; MIFARE auth |
| `t-mixrf-codec2-433` | `fsk-walkie-talkie` | CC1101 FSK + Codec2 3200 walkie-talkie at 433 MHz |
| `t-mixrf-dual-band-lora` | `deauth-detector` | `alert_t` backhaul on 868+2.4 GHz, automatic band selection |

---

## References

- LILYGO T-MixRF GitHub: https://github.com/Xinyuan-LilyGO/T-MixRF
- RadioLib LR11x0 opcodes: https://github.com/jgromes/RadioLib/blob/master/src/modules/LR11x0/LR11x0_commands.h
- CC1101 datasheet: Texas Instruments SWRS061H
- nRF24L01+ spec: Nordic Semiconductor PS v1.0
- ST25R3916 AN5538 (SPI protocol)
