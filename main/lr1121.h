#pragma once
#include "driver/spi_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/* ── LoRa modulation parameters ───────────────────────────────────────── */
typedef enum {
    LR1121_BW_62_5   = 0x03, /* 62.5 kHz */
    LR1121_BW_125    = 0x04, /* 125 kHz */
    LR1121_BW_250    = 0x05, /* 250 kHz */
    LR1121_BW_500    = 0x06, /* 500 kHz */
} lr1121_bw_t;

typedef enum {
    LR1121_CR_4_5 = 0x01,
    LR1121_CR_4_6 = 0x02,
    LR1121_CR_4_7 = 0x03,
    LR1121_CR_4_8 = 0x04,
} lr1121_cr_t;

typedef struct {
    spi_device_handle_t spi;
    uint32_t            freq_hz;
    uint8_t             sf;       /* spreading factor 5–12 */
    lr1121_bw_t         bw;
    lr1121_cr_t         cr;
    int8_t              power_dbm;
} lr1121_t;

/*
 * Initialise LR1121 in LoRa mode.
 * freq_hz  : e.g. 868000000 (sub-GHz) or 2400000000 (2.4 GHz)
 * sf       : spreading factor 5–12
 * bw       : LR1121_BW_*
 * cr       : LR1121_CR_*
 * power_dbm: output power in dBm (sub-GHz: −17…+22, 2.4 GHz: −18…+13)
 *
 * The board's RF switch (PIN_LR1121_SWITCH) is driven by this driver:
 *   HIGH → sub-GHz path, LOW → 2.4 GHz path.
 */
esp_err_t lr1121_init(lr1121_t *dev, uint32_t freq_hz,
                       uint8_t sf, lr1121_bw_t bw, lr1121_cr_t cr,
                       int8_t power_dbm);

/* Write payload to radio buffer and transmit. Blocks until TX_DONE or timeout. */
esp_err_t lr1121_transmit(lr1121_t *dev, const uint8_t *data, uint8_t len,
                           uint32_t timeout_ms);

/* Enter continuous RX mode and wait for a packet or timeout. */
esp_err_t lr1121_receive(lr1121_t *dev, uint8_t *buf, uint8_t *len,
                          int8_t *rssi, int8_t *snr, uint32_t timeout_ms);

/* Place chip in standby-RC (lowest-power standby). */
esp_err_t lr1121_standby(lr1121_t *dev);

/* Hardware reset (RST pin, active LOW). */
esp_err_t lr1121_reset(lr1121_t *dev);

/* Read chip firmware version: hw_version, use_case, fw_major, fw_minor. */
esp_err_t lr1121_get_version(lr1121_t *dev,
                              uint8_t *hw, uint8_t *use_case,
                              uint8_t *fw_maj, uint8_t *fw_min);
