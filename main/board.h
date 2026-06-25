#pragma once

/*
 * LILYGO T-MixRF [H775] — ESP32-S3-WROOM-1-N16R8 pin definitions.
 *
 * IMPORTANT: GPIO33-37 are shared with the OPI PSRAM interface inside the
 * WROOM module. They are usable as ordinary GPIO only when CONFIG_SPIRAM=n.
 * The sdkconfig.defaults in this project enforces that setting.
 *
 * Reference: https://github.com/Xinyuan-LilyGO/T-MixRF  (pin.h)
 */

/* ── Shared SPI bus ─────────────────────────────────────────────────── */
#define PIN_SPI_SCK     17
#define PIN_SPI_MOSI    15
#define PIN_SPI_MISO    16

/* ── LR1121 dual-band LoRa (Sub-GHz + 2.4 GHz) ──────────────────────
 *   DIO9 is the interrupt output (TX_DONE / RX_DONE / TIMEOUT).
 *   BUSY stays HIGH while the chip is processing a command.
 *   RST is active-LOW hardware reset (hold LOW ≥100 µs).
 *   SWITCH is an external GPIO driving the board's RF antenna switch;
 *   it is NOT an LR1121 DIO pin — drive it manually before TX/RX.
 */
#define PIN_LR1121_CS       33
#define PIN_LR1121_DIO9     9
#define PIN_LR1121_RST      37
#define PIN_LR1121_BUSY     6
#define PIN_LR1121_SWITCH   7   /* RF antenna switch: HIGH=sub-GHz, LOW=2.4 GHz */

/* ── CC1101 sub-GHz transceiver ──────────────────────────────────────
 *   GDO0 is configured as packet-sync / end-of-packet signal.
 *   GDO2 is configured as carrier-sense / TX FIFO threshold.
 *   SW0/SW1 select the antenna band filter (see cc1101.h).
 */
#define PIN_CC1101_CS       11
#define PIN_CC1101_GDO0     14
#define PIN_CC1101_GDO2     12
#define PIN_CC1101_SW0      13
#define PIN_CC1101_SW1      10

/* ── nRF24L01 2.4 GHz transceiver ───────────────────────────────────
 *   CE (chip enable) controls RX/TX mode.
 *   IRQ is active-LOW — fires on TX_DS, RX_DR, MAX_RT.
 */
#define PIN_NRF24_CS        34
#define PIN_NRF24_CE        4
#define PIN_NRF24_IRQ       5

/* ── ST25R3916 NFC reader ────────────────────────────────────────────
 *   BSS = chip-select (active LOW).
 *   EN  = chip enable (active HIGH — powers analog front-end).
 */
#define PIN_NFC_CS          36
#define PIN_NFC_EN          35
