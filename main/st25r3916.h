#pragma once
#include "driver/spi_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/* ── ST25R3916 register addresses (partial — see AN5538) ───────────── */
#define ST25_REG_IO_CONF1   0x00
#define ST25_REG_IO_CONF2   0x01
#define ST25_REG_OP_CONTROL 0x02
#define ST25_REG_MODE_DEF   0x03
#define ST25_REG_BIT_RATE   0x04
#define ST25_REG_NFC_IP1    0x05
#define ST25_REG_NFC_IP2    0x06
#define ST25_REG_IC_IDENT   0x3F   /* Product ID: 0x09 for ST25R3916 */

/* ── OP_CONTROL bits ──────────────────────────────────────────────────── */
#define ST25_OP_EN          (1 << 7)  /* chip enable (alias — also driven by EN pin) */
#define ST25_OP_RX_EN       (1 << 6)  /* receiver enable */
#define ST25_OP_TX_EN       (1 << 3)  /* transmitter enable */

/* ── MODE_DEF bits ────────────────────────────────────────────────────── */
#define ST25_MODE_ISO14443A 0x00
#define ST25_MODE_ISO14443B 0x01
#define ST25_MODE_NFCIP1    0x03
#define ST25_MODE_ISO15693  0x04

/* ── Direct commands ──────────────────────────────────────────────────── */
#define ST25_CMD_SET_DEFAULT    0xC1
#define ST25_CMD_STOP           0xC2
#define ST25_CMD_TRANSMIT_WITH_CRC  0xC4
#define ST25_CMD_RECEIVE        0xC8
#define ST25_CMD_RESET_RXGAIN   0xCA
#define ST25_CMD_ADJUST_REGULATORS 0xCB
#define ST25_CMD_CALIBRATE_MODULATION 0xCC

typedef struct {
    spi_device_handle_t spi;
} st25r3916_t;

/* Power on EN pin, reset chip, verify IC identity, configure ISO14443A. */
esp_err_t st25r3916_init(st25r3916_t *dev);

/* Read a single register. */
uint8_t st25r3916_read_reg(st25r3916_t *dev, uint8_t addr);

/* Write a single register. */
void st25r3916_write_reg(st25r3916_t *dev, uint8_t addr, uint8_t val);

/* Issue a direct command (0xC0–0xFF range). */
void st25r3916_cmd(st25r3916_t *dev, uint8_t cmd);

/*
 * ISO14443-A card detect + anti-collision (Type A, 106 kbps).
 * Fills uid[10] and sets *uid_len. Returns ESP_OK if a card is present.
 */
esp_err_t st25r3916_detect_iso14443a(st25r3916_t *dev,
                                      uint8_t *uid, uint8_t *uid_len);
