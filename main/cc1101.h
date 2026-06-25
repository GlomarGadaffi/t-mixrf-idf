#pragma once
#include "driver/spi_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Supported frequency bands ───────────────────────────────────────
 * SW0/SW1 control the on-board SAW filter / antenna switch.
 *   315 MHz : SW1=1, SW0=0
 *   433 MHz : SW1=1, SW0=1
 *   868 MHz : SW1=0, SW0=1
 *   915 MHz : SW1=0, SW0=1   (same filter path as 868)
 */
typedef enum {
    CC1101_BAND_315 = 0,
    CC1101_BAND_433,
    CC1101_BAND_868,
    CC1101_BAND_915,
} cc1101_band_t;

typedef enum {
    CC1101_MOD_OOK = 0,
    CC1101_MOD_FSK2,
} cc1101_mod_t;

typedef struct {
    spi_device_handle_t spi;
    cc1101_band_t       band;
    cc1101_mod_t        modulation;
} cc1101_t;

/* Initialise device, apply register preset, return handle via *dev. */
esp_err_t cc1101_init(cc1101_t *dev, cc1101_band_t band, cc1101_mod_t mod);

/* Switch band; reconfigures FREQ registers and SW0/SW1. */
esp_err_t cc1101_set_band(cc1101_t *dev, cc1101_band_t band);

/* Transmit `len` bytes from `data` (variable-length packet mode). */
esp_err_t cc1101_transmit(cc1101_t *dev, const uint8_t *data, uint8_t len);

/* Strobe SRX and block until a packet arrives on GDO0 or timeout_ms elapses.
 * Returns ESP_ERR_TIMEOUT when no packet received within timeout_ms. */
esp_err_t cc1101_receive(cc1101_t *dev, uint8_t *buf, uint8_t *len,
                         int8_t *rssi_dbm, uint32_t timeout_ms);

/* Place chip in IDLE state. */
esp_err_t cc1101_idle(cc1101_t *dev);

/* Read a configuration register (0x00–0x2E). */
uint8_t cc1101_read_reg(cc1101_t *dev, uint8_t addr);

/* Write a configuration register. */
void cc1101_write_reg(cc1101_t *dev, uint8_t addr, uint8_t value);
