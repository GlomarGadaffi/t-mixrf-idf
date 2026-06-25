#include "cc1101.h"
#include "board.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "cc1101";

/* ── SPI header byte fields ──────────────────────────────────────────── */
#define CC1101_READ         0x80
#define CC1101_BURST        0x40
#define CC1101_WRITE        0x00

/* ── Configuration register addresses (0x00–0x2E) ──────────────────── */
#define REG_IOCFG2          0x00
#define REG_IOCFG1          0x01
#define REG_IOCFG0          0x02
#define REG_FIFOTHR         0x03
#define REG_SYNC1           0x04
#define REG_SYNC0           0x05
#define REG_PKTLEN          0x06
#define REG_PKTCTRL1        0x07
#define REG_PKTCTRL0        0x08
#define REG_ADDR            0x09
#define REG_CHANNR          0x0A
#define REG_FSCTRL1         0x0B
#define REG_FSCTRL0         0x0C
#define REG_FREQ2           0x0D
#define REG_FREQ1           0x0E
#define REG_FREQ0           0x0F
#define REG_MDMCFG4         0x10
#define REG_MDMCFG3         0x11
#define REG_MDMCFG2         0x12
#define REG_MDMCFG1         0x13
#define REG_MDMCFG0         0x14
#define REG_DEVIATN         0x15
#define REG_MCSM2           0x16
#define REG_MCSM1           0x17
#define REG_MCSM0           0x18
#define REG_FOCCFG          0x19
#define REG_BSCFG           0x1A
#define REG_AGCCTRL2        0x1B
#define REG_AGCCTRL1        0x1C
#define REG_AGCCTRL0        0x1D
#define REG_WOREVT1         0x1E
#define REG_WOREVT0         0x1F
#define REG_WORCTRL         0x20
#define REG_FREND1          0x21
#define REG_FREND0          0x22
#define REG_FSCAL3          0x23
#define REG_FSCAL2          0x24
#define REG_FSCAL1          0x25
#define REG_FSCAL0          0x26
#define REG_RCCTRL1         0x27
#define REG_RCCTRL0         0x28
#define REG_TEST2           0x2C
#define REG_TEST1           0x2D
#define REG_TEST0           0x2E

/* ── Status registers (read with 0xC0|addr) ─────────────────────────── */
#define REG_PARTNUM         0x30
#define REG_VERSION         0x31
#define REG_RSSI            0x34
#define REG_MARCSTATE       0x35
#define REG_PKTSTATUS       0x38
#define REG_TXBYTES         0x3A
#define REG_RXBYTES         0x3B

/* ── Strobes ──────────────────────────────────────────────────────────── */
#define STROBE_SRES         0x30   /* reset */
#define STROBE_SCAL         0x33   /* calibrate synthesizer */
#define STROBE_SRX          0x34   /* enter RX */
#define STROBE_STX          0x35   /* enter TX */
#define STROBE_SIDLE        0x36   /* enter IDLE */
#define STROBE_SFRX         0x3A   /* flush RX FIFO */
#define STROBE_SFTX         0x3B   /* flush TX FIFO */
#define STROBE_SNOP         0x3D   /* no-operation (read chip status) */

/* ── PATABLE / FIFO ──────────────────────────────────────────────────── */
#define ADDR_PATABLE        0x3E
#define ADDR_TXFIFO         0x3F
#define ADDR_RXFIFO         0x3F

/* ── Frequency presets (FREQ2, FREQ1, FREQ0) for 26 MHz crystal ──────
 *   Formula: FREQ = f_Hz * 2^16 / 26e6
 */
static const uint8_t FREQ_315[3] = { 0x0C, 0x1D, 0x89 }; /* ~315.000 MHz */
static const uint8_t FREQ_433[3] = { 0x10, 0xA7, 0x62 }; /* ~433.000 MHz */
static const uint8_t FREQ_868[3] = { 0x21, 0x62, 0x76 }; /* ~868.000 MHz */
static const uint8_t FREQ_915[3] = { 0x23, 0x31, 0x3B }; /* ~914.999 MHz */

/* ── SPI helpers ─────────────────────────────────────────────────────── */
static void spi_xfer(spi_device_handle_t spi, uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
}

static uint8_t strobe(spi_device_handle_t spi, uint8_t cmd)
{
    uint8_t tx = cmd, rx = 0;
    spi_xfer(spi, &tx, &rx, 1);
    return rx; /* chip status byte */
}

void cc1101_write_reg(cc1101_t *dev, uint8_t addr, uint8_t value)
{
    uint8_t tx[2] = { CC1101_WRITE | addr, value };
    spi_xfer(dev->spi, tx, NULL, 2);
}

uint8_t cc1101_read_reg(cc1101_t *dev, uint8_t addr)
{
    uint8_t tx[2] = { CC1101_READ | addr, 0x00 };
    uint8_t rx[2] = { 0 };
    spi_xfer(dev->spi, tx, rx, 2);
    return rx[1];
}

/* Burst-write N bytes starting at addr. */
static void write_burst(cc1101_t *dev, uint8_t addr, const uint8_t *data, uint8_t n)
{
    uint8_t tx[n + 1];
    tx[0] = CC1101_WRITE | CC1101_BURST | addr;
    memcpy(&tx[1], data, n);
    spi_xfer(dev->spi, tx, NULL, n + 1);
}

/* Burst-read N bytes starting at addr. */
static void read_burst(cc1101_t *dev, uint8_t addr, uint8_t *dst, uint8_t n)
{
    uint8_t tx[n + 1];
    uint8_t rx[n + 1];
    memset(tx, 0, sizeof(tx));
    tx[0] = CC1101_READ | CC1101_BURST | addr;
    spi_xfer(dev->spi, tx, rx, n + 1);
    memcpy(dst, &rx[1], n);
}

/* Read a status register (requires burst bit per CC1101 spec §10.4). */
static uint8_t read_status(cc1101_t *dev, uint8_t addr)
{
    uint8_t tx[2] = { CC1101_READ | CC1101_BURST | addr, 0x00 };
    uint8_t rx[2] = { 0 };
    spi_xfer(dev->spi, tx, rx, 2);
    return rx[1];
}

/* ── Band switch ──────────────────────────────────────────────────────── */
static void apply_band_switch(cc1101_band_t band)
{
    /* SW1=GPIO10, SW0=GPIO13 */
    int sw1, sw0;
    switch (band) {
    case CC1101_BAND_315: sw1 = 1; sw0 = 0; break;
    case CC1101_BAND_433: sw1 = 1; sw0 = 1; break;
    case CC1101_BAND_868: /* fall-through */
    case CC1101_BAND_915: sw1 = 0; sw0 = 1; break;
    default:              sw1 = 0; sw0 = 0; break;
    }
    gpio_set_level(PIN_CC1101_SW1, sw1);
    gpio_set_level(PIN_CC1101_SW0, sw0);
}

static void apply_freq(cc1101_t *dev, cc1101_band_t band)
{
    const uint8_t *f;
    switch (band) {
    case CC1101_BAND_315: f = FREQ_315; break;
    case CC1101_BAND_433: f = FREQ_433; break;
    case CC1101_BAND_868: f = FREQ_868; break;
    case CC1101_BAND_915: f = FREQ_915; break;
    default:              f = FREQ_433; break;
    }
    cc1101_write_reg(dev, REG_FREQ2, f[0]);
    cc1101_write_reg(dev, REG_FREQ1, f[1]);
    cc1101_write_reg(dev, REG_FREQ0, f[2]);
}

/* ── Register preset tables ──────────────────────────────────────────── */

/*
 * OOK at 433 MHz, 1.2 kbps, 58 kHz BW, variable packet length with CRC.
 * Derived from TI SmartRF Studio 7 preset (CC1101 433 MHz OOK 1.2 kbps).
 *
 * GDO0 (GPIO14): asserts when sync word received, deasserts end-of-packet.
 * GDO2 (GPIO12): carrier-sense / TX FIFO above threshold.
 * PATABLE[0]=0x00 (off), PATABLE[1]=0xC0 (~+10 dBm on).
 */
static const uint8_t OOK_433_REGS[][2] = {
    { REG_IOCFG2,   0x0D }, /* GDO2: carrier sense (deasserts when channel is clear) */
    { REG_IOCFG0,   0x06 }, /* GDO0: asserts when sync word received */
    { REG_FIFOTHR,  0x47 }, /* TX≥33, RX≥32 bytes threshold */
    { REG_SYNC1,    0xD3 }, /* Sync word high */
    { REG_SYNC0,    0x91 }, /* Sync word low */
    { REG_PKTLEN,   0xFF }, /* Max 255 bytes */
    { REG_PKTCTRL1, 0x04 }, /* Append status, no addr check */
    { REG_PKTCTRL0, 0x05 }, /* Variable length, CRC enabled, no whitening */
    { REG_CHANNR,   0x00 },
    { REG_FSCTRL1,  0x06 }, /* IF = 152 kHz */
    { REG_FSCTRL0,  0x00 },
    /* FREQ set later by apply_freq() */
    { REG_MDMCFG4,  0x87 }, /* BW = 58 kHz, DRATE_E = 7 */
    { REG_MDMCFG3,  0x83 }, /* DRATE_M = 131 → ~1.2 kbps */
    { REG_MDMCFG2,  0x33 }, /* OOK, 16/16 sync word, no Manchester */
    { REG_MDMCFG1,  0x22 }, /* No FEC, 4 preamble bytes */
    { REG_MDMCFG0,  0xF8 },
    { REG_DEVIATN,  0x15 }, /* N/A for OOK */
    { REG_MCSM1,    0x30 }, /* TX→IDLE, RX→IDLE */
    { REG_MCSM0,    0x18 }, /* Auto-cal on IDLE→RX/TX */
    { REG_FOCCFG,   0x14 },
    { REG_BSCFG,    0x6C },
    { REG_AGCCTRL2, 0x07 },
    { REG_AGCCTRL1, 0x00 },
    { REG_AGCCTRL0, 0x91 },
    { REG_WORCTRL,  0xFB },
    { REG_FREND1,   0x56 },
    { REG_FREND0,   0x11 }, /* OOK: use PATABLE index 1 for "on" */
    { REG_FSCAL3,   0xE9 },
    { REG_FSCAL2,   0x2A },
    { REG_FSCAL1,   0x00 },
    { REG_FSCAL0,   0x1F },
    { REG_TEST2,    0x81 },
    { REG_TEST1,    0x35 },
    { REG_TEST0,    0x09 },
};

/* 2-FSK at 433/868/915 MHz, 4.8 kbps, 162 kHz BW, variable packet + CRC. */
static const uint8_t FSK_REGS[][2] = {
    { REG_IOCFG2,   0x0D },
    { REG_IOCFG0,   0x06 },
    { REG_FIFOTHR,  0x47 },
    { REG_SYNC1,    0xD3 },
    { REG_SYNC0,    0x91 },
    { REG_PKTLEN,   0xFF },
    { REG_PKTCTRL1, 0x04 },
    { REG_PKTCTRL0, 0x05 },
    { REG_CHANNR,   0x00 },
    { REG_FSCTRL1,  0x08 },
    { REG_FSCTRL0,  0x00 },
    { REG_MDMCFG4,  0x7B }, /* BW = 162 kHz, DRATE_E = 11 */
    { REG_MDMCFG3,  0x83 }, /* DRATE_M = 131 → ~4.8 kbps @ DRATE_E=7 */
    { REG_MDMCFG2,  0x03 }, /* 2-FSK, 16/16 sync, no Manchester */
    { REG_MDMCFG1,  0x22 },
    { REG_MDMCFG0,  0xF8 },
    { REG_DEVIATN,  0x47 }, /* ±47.6 kHz deviation */
    { REG_MCSM1,    0x30 },
    { REG_MCSM0,    0x18 },
    { REG_FOCCFG,   0x1D },
    { REG_BSCFG,    0x1C },
    { REG_AGCCTRL2, 0xC7 },
    { REG_AGCCTRL1, 0x00 },
    { REG_AGCCTRL0, 0xB2 },
    { REG_WORCTRL,  0xFB },
    { REG_FREND1,   0xB6 },
    { REG_FREND0,   0x10 }, /* FSK: use PATABLE index 0 */
    { REG_FSCAL3,   0xEA },
    { REG_FSCAL2,   0x2A },
    { REG_FSCAL1,   0x00 },
    { REG_FSCAL0,   0x1F },
    { REG_TEST2,    0x81 },
    { REG_TEST1,    0x35 },
    { REG_TEST0,    0x09 },
};

/* ── Public API ──────────────────────────────────────────────────────── */

esp_err_t cc1101_init(cc1101_t *dev, cc1101_band_t band, cc1101_mod_t mod)
{
    /* SW0/SW1 band select GPIOs */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_CC1101_SW0) | (1ULL << PIN_CC1101_SW1),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    /* GDO0 as input (poll-based receive) */
    gpio_config_t gdo0 = {
        .pin_bit_mask = (1ULL << PIN_CC1101_GDO0),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gdo0));

    /* Register SPI device — CC1101 uses SPI Mode 0, max 10 MHz */
    spi_device_interface_config_t dcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode           = 0,
        .spics_io_num   = PIN_CC1101_CS,
        .queue_size     = 4,
        .flags          = 0,
        .pre_cb         = NULL,
        .post_cb        = NULL,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dcfg, &dev->spi));
    dev->band       = band;
    dev->modulation = mod;

    /* Reset via SRES strobe */
    strobe(dev->spi, STROBE_SRES);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Verify chip identity (PARTNUM should be 0x00, VERSION typically 0x14) */
    uint8_t pn = read_status(dev, REG_PARTNUM);
    uint8_t ver = read_status(dev, REG_VERSION);
    ESP_LOGI(TAG, "CC1101 PARTNUM=0x%02X VERSION=0x%02X", pn, ver);
    if (pn != 0x00) {
        ESP_LOGE(TAG, "Unexpected PARTNUM — check wiring");
        return ESP_ERR_NOT_FOUND;
    }

    /* Apply register preset */
    const uint8_t (*regs)[2];
    size_t count;
    if (mod == CC1101_MOD_OOK) {
        regs  = OOK_433_REGS;
        count = sizeof(OOK_433_REGS) / sizeof(OOK_433_REGS[0]);
    } else {
        regs  = FSK_REGS;
        count = sizeof(FSK_REGS) / sizeof(FSK_REGS[0]);
    }
    for (size_t i = 0; i < count; i++) {
        cc1101_write_reg(dev, regs[i][0], regs[i][1]);
    }

    /* Frequency and band switch */
    apply_freq(dev, band);
    apply_band_switch(band);

    /* PATABLE for OOK: 0x00 = off, 0xC0 ≈ +10 dBm on */
    if (mod == CC1101_MOD_OOK) {
        uint8_t patable[8] = { 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        write_burst(dev, ADDR_PATABLE, patable, 8);
    } else {
        uint8_t patable[1] = { 0xC0 }; /* ~+10 dBm FSK */
        write_burst(dev, ADDR_PATABLE, patable, 1);
    }

    /* Calibrate synthesizer */
    cc1101_idle(dev);
    strobe(dev->spi, STROBE_SCAL);
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGI(TAG, "CC1101 ready — band=%d mod=%d", band, mod);
    return ESP_OK;
}

esp_err_t cc1101_set_band(cc1101_t *dev, cc1101_band_t band)
{
    cc1101_idle(dev);
    dev->band = band;
    apply_freq(dev, band);
    apply_band_switch(band);
    strobe(dev->spi, STROBE_SCAL);
    vTaskDelay(pdMS_TO_TICKS(5));
    return ESP_OK;
}

esp_err_t cc1101_idle(cc1101_t *dev)
{
    strobe(dev->spi, STROBE_SIDLE);
    strobe(dev->spi, STROBE_SFRX);
    strobe(dev->spi, STROBE_SFTX);
    return ESP_OK;
}

esp_err_t cc1101_transmit(cc1101_t *dev, const uint8_t *data, uint8_t len)
{
    cc1101_idle(dev);

    /* Write length byte + payload into TX FIFO */
    uint8_t hdr = CC1101_WRITE | CC1101_BURST | ADDR_TXFIFO;
    uint8_t tx_buf[len + 2];
    tx_buf[0] = hdr;
    tx_buf[1] = len;           /* length byte (variable packet mode) */
    memcpy(&tx_buf[2], data, len);
    spi_xfer(dev->spi, tx_buf, NULL, len + 2);

    /* Start transmission */
    strobe(dev->spi, STROBE_STX);

    /* Poll GDO0: HIGH while transmitting, goes LOW at end-of-packet */
    uint32_t deadline = 1000; /* ms */
    while (!gpio_get_level(PIN_CC1101_GDO0) && deadline) {
        vTaskDelay(pdMS_TO_TICKS(1));
        deadline--;
    }
    while (gpio_get_level(PIN_CC1101_GDO0) && deadline) {
        vTaskDelay(pdMS_TO_TICKS(1));
        deadline--;
    }

    cc1101_idle(dev);
    return (deadline == 0) ? ESP_ERR_TIMEOUT : ESP_OK;
}

esp_err_t cc1101_receive(cc1101_t *dev, uint8_t *buf, uint8_t *len,
                         int8_t *rssi_dbm, uint32_t timeout_ms)
{
    cc1101_idle(dev);
    strobe(dev->spi, STROBE_SRX);

    /* Wait for GDO0 to assert (sync received) then deassert (packet done) */
    uint32_t t = timeout_ms;
    while (!gpio_get_level(PIN_CC1101_GDO0) && t) {
        vTaskDelay(pdMS_TO_TICKS(1));
        t--;
    }
    if (t == 0) {
        cc1101_idle(dev);
        return ESP_ERR_TIMEOUT;
    }
    while (gpio_get_level(PIN_CC1101_GDO0)) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* Check RX FIFO byte count */
    uint8_t rxbytes = read_status(dev, REG_RXBYTES);
    if (rxbytes == 0) {
        cc1101_idle(dev);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Read: length byte, payload, then 2 appended status bytes (RSSI, LQI) */
    uint8_t pktlen = cc1101_read_reg(dev, ADDR_RXFIFO);
    if (pktlen > 61 || pktlen == 0) { /* sanity */
        cc1101_idle(dev);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t payload_and_status[pktlen + 2];
    read_burst(dev, ADDR_RXFIFO, payload_and_status, pktlen + 2);

    memcpy(buf, payload_and_status, pktlen);
    *len = pktlen;

    /* RSSI conversion: RSSIdBm = RSSI_dec/2 − 74  (CC1101 §17.3) */
    uint8_t rssi_raw = payload_and_status[pktlen];
    *rssi_dbm = (rssi_raw >= 128) ? ((int8_t)rssi_raw / 2 - 74)
                                  : (rssi_raw / 2 + 128 - 74);

    cc1101_idle(dev);
    return ESP_OK;
}
