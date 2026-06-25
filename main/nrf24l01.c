#include "nrf24l01.h"
#include "board.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "nrf24";

/* ── SPI commands ─────────────────────────────────────────────────────── */
#define CMD_R_REGISTER      0x00   /* | 5-bit reg addr */
#define CMD_W_REGISTER      0x20   /* | 5-bit reg addr */
#define CMD_R_RX_PAYLOAD    0x61
#define CMD_W_TX_PAYLOAD    0xA0
#define CMD_FLUSH_TX        0xE1
#define CMD_FLUSH_RX        0xE2
#define CMD_REUSE_TX_PL     0xE3
#define CMD_R_RX_PL_WID     0x60
#define CMD_W_TX_PLD_NO_ACK 0xB0
#define CMD_NOP             0xFF

/* ── Register addresses ───────────────────────────────────────────────── */
#define REG_CONFIG          0x00
#define REG_EN_AA           0x01
#define REG_EN_RXADDR       0x02
#define REG_SETUP_AW        0x03
#define REG_SETUP_RETR      0x04
#define REG_RF_CH           0x05
#define REG_RF_SETUP        0x06
#define REG_STATUS          0x07
#define REG_OBSERVE_TX      0x08
#define REG_RPD             0x09
#define REG_RX_ADDR_P0      0x0A
#define REG_RX_ADDR_P1      0x0B
#define REG_TX_ADDR         0x10
#define REG_RX_PW_P0        0x11
#define REG_RX_PW_P1        0x12
#define REG_FIFO_STATUS     0x17
#define REG_DYNPD           0x1C
#define REG_FEATURE         0x1D

/* ── CONFIG register bits ─────────────────────────────────────────────── */
#define CFG_MASK_RX_DR  (1 << 6)
#define CFG_MASK_TX_DS  (1 << 5)
#define CFG_MASK_MAX_RT (1 << 4)
#define CFG_EN_CRC      (1 << 3)
#define CFG_CRCO        (1 << 2)   /* 1 = 2-byte CRC */
#define CFG_PWR_UP      (1 << 1)
#define CFG_PRIM_RX     (1 << 0)

/* ── STATUS register bits ─────────────────────────────────────────────── */
#define STA_RX_DR       (1 << 6)
#define STA_TX_DS       (1 << 5)
#define STA_MAX_RT      (1 << 4)
#define STA_TX_FULL     (1 << 0)
#define STA_RX_P_NO     (0x0E)     /* bits 3:1 */
#define STA_RX_EMPTY    (0x0E)     /* 0b1110 = no pipe */

/* ── FIFO_STATUS bits ─────────────────────────────────────────────────── */
#define FIFO_TX_EMPTY   (1 << 4)
#define FIFO_RX_EMPTY   (1 << 0)

/* ── RF_SETUP bits ────────────────────────────────────────────────────── */
/* Data rate: [RF_DR_LOW(bit5), RF_DR_HIGH(bit3)] → 00=1Mbps, 01=2Mbps, 10=250kbps */
#define RF_SETUP_DR_LOW     (1 << 5)
#define RF_SETUP_DR_HIGH    (1 << 3)
#define RF_SETUP_PWR_SHIFT  1       /* bits 2:1 */

/* ── SPI helpers ─────────────────────────────────────────────────────── */
static void spi_xfer(spi_device_handle_t spi, const uint8_t *tx,
                     uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
}

uint8_t nrf24_read_reg(nrf24_t *dev, uint8_t reg)
{
    uint8_t tx[2] = { CMD_R_REGISTER | (reg & 0x1F), CMD_NOP };
    uint8_t rx[2] = { 0 };
    spi_xfer(dev->spi, tx, rx, 2);
    return rx[1];
}

void nrf24_write_reg(nrf24_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { CMD_W_REGISTER | (reg & 0x1F), val };
    spi_xfer(dev->spi, tx, NULL, 2);
}

static void write_reg_multi(nrf24_t *dev, uint8_t reg,
                             const uint8_t *data, uint8_t len)
{
    uint8_t tx[len + 1];
    tx[0] = CMD_W_REGISTER | (reg & 0x1F);
    memcpy(&tx[1], data, len);
    spi_xfer(dev->spi, tx, NULL, len + 1);
}

static void read_reg_multi(nrf24_t *dev, uint8_t reg, uint8_t *dst, uint8_t len)
{
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];
    memset(tx, CMD_NOP, sizeof(tx));
    tx[0] = CMD_R_REGISTER | (reg & 0x1F);
    spi_xfer(dev->spi, tx, rx, len + 1);
    memcpy(dst, &rx[1], len);
}

static uint8_t get_status(nrf24_t *dev)
{
    uint8_t tx = CMD_NOP, rx = 0;
    spi_xfer(dev->spi, &tx, &rx, 1);
    return rx;
}

static void ce_high(void) { gpio_set_level(PIN_NRF24_CE, 1); }
static void ce_low(void)  { gpio_set_level(PIN_NRF24_CE, 0); }

/* ── Common init shared by TX and RX paths ────────────────────────────── */
static esp_err_t nrf24_common_init(nrf24_t *dev, uint8_t channel,
                                    const uint8_t pipe_addr[NRF24_ADDR_WIDTH],
                                    uint8_t payload_w,
                                    nrf24_datarate_t dr, nrf24_pa_t pa)
{
    /* CE and IRQ GPIO config */
    gpio_config_t io_ce = {
        .pin_bit_mask = (1ULL << PIN_NRF24_CE),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_ce));
    ce_low();

    gpio_config_t io_irq = {
        .pin_bit_mask = (1ULL << PIN_NRF24_IRQ),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_irq));

    /* Register SPI device — nRF24L01+ Mode 0, max 10 MHz */
    spi_device_interface_config_t dcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode           = 0,
        .spics_io_num   = PIN_NRF24_CS,
        .queue_size     = 4,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dcfg, &dev->spi));

    vTaskDelay(pdMS_TO_TICKS(5)); /* power-on delay */

    /* Power down first, clear old state */
    nrf24_write_reg(dev, REG_CONFIG, CFG_EN_CRC | CFG_CRCO);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Auto-ACK on pipe 0 only */
    nrf24_write_reg(dev, REG_EN_AA, 0x01);

    /* Enable RX pipe 0 */
    nrf24_write_reg(dev, REG_EN_RXADDR, 0x01);

    /* 5-byte address width */
    nrf24_write_reg(dev, REG_SETUP_AW, 0x03);

    /* Auto-retransmit: 500 µs delay, 10 retries */
    nrf24_write_reg(dev, REG_SETUP_RETR, (0x01 << 4) | 0x0A);

    /* RF channel */
    nrf24_write_reg(dev, REG_RF_CH, channel & 0x7F);

    /* RF setup: data rate + PA */
    uint8_t rf_setup = (uint8_t)(pa & 0x03) << RF_SETUP_PWR_SHIFT;
    switch (dr) {
    case NRF24_DR_2MBPS:   rf_setup |= RF_SETUP_DR_HIGH; break;
    case NRF24_DR_250KBPS: rf_setup |= RF_SETUP_DR_LOW;  break;
    default:               break; /* 1 Mbps: both bits 0 */
    }
    nrf24_write_reg(dev, REG_RF_SETUP, rf_setup);

    /* Pipe addresses */
    write_reg_multi(dev, REG_TX_ADDR,    pipe_addr, NRF24_ADDR_WIDTH);
    write_reg_multi(dev, REG_RX_ADDR_P0, pipe_addr, NRF24_ADDR_WIDTH); /* ACK pipe */

    /* Payload width */
    if (payload_w == 0) {
        /* Dynamic payload length */
        nrf24_write_reg(dev, REG_DYNPD,   0x01);
        nrf24_write_reg(dev, REG_FEATURE, 0x04); /* EN_DPL */
    } else {
        nrf24_write_reg(dev, REG_RX_PW_P0, payload_w);
        nrf24_write_reg(dev, REG_DYNPD,    0x00);
        nrf24_write_reg(dev, REG_FEATURE,  0x00);
    }

    /* Clear status flags */
    nrf24_write_reg(dev, REG_STATUS, STA_RX_DR | STA_TX_DS | STA_MAX_RT);

    /* Flush FIFOs */
    uint8_t cmd;
    cmd = CMD_FLUSH_TX; spi_xfer(dev->spi, &cmd, NULL, 1);
    cmd = CMD_FLUSH_RX; spi_xfer(dev->spi, &cmd, NULL, 1);

    return ESP_OK;
}

esp_err_t nrf24_init_tx(nrf24_t *dev, uint8_t channel,
                         const uint8_t pipe_addr[NRF24_ADDR_WIDTH],
                         uint8_t payload_w, nrf24_datarate_t dr, nrf24_pa_t pa)
{
    dev->prx_mode = false;
    ESP_ERROR_CHECK(nrf24_common_init(dev, channel, pipe_addr, payload_w, dr, pa));

    /* Power up in PTX mode */
    nrf24_write_reg(dev, REG_CONFIG,
                    CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP);
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGI(TAG, "nRF24 TX ready (ch=%d dr=%d pa=%d)", channel, dr, pa);
    return ESP_OK;
}

esp_err_t nrf24_init_rx(nrf24_t *dev, uint8_t channel,
                         const uint8_t pipe_addr[NRF24_ADDR_WIDTH],
                         uint8_t payload_w, nrf24_datarate_t dr, nrf24_pa_t pa)
{
    dev->prx_mode = true;
    ESP_ERROR_CHECK(nrf24_common_init(dev, channel, pipe_addr, payload_w, dr, pa));

    /* Power up in PRX mode */
    nrf24_write_reg(dev, REG_CONFIG,
                    CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP | CFG_PRIM_RX);
    vTaskDelay(pdMS_TO_TICKS(5));
    ce_high(); /* Start listening */

    ESP_LOGI(TAG, "nRF24 RX listening (ch=%d dr=%d pa=%d)", channel, dr, pa);
    return ESP_OK;
}

esp_err_t nrf24_transmit(nrf24_t *dev, const uint8_t *data, uint8_t len)
{
    if (len > NRF24_MAX_PAYLOAD) len = NRF24_MAX_PAYLOAD;

    /* Switch to PTX if currently in PRX */
    if (dev->prx_mode) {
        ce_low();
        nrf24_write_reg(dev, REG_CONFIG, CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* Load TX payload */
    uint8_t tx_buf[len + 1];
    tx_buf[0] = CMD_W_TX_PAYLOAD;
    memcpy(&tx_buf[1], data, len);
    spi_xfer(dev->spi, tx_buf, NULL, len + 1);

    /* Pulse CE for ≥10 µs to start transmission */
    ce_high();
    esp_rom_delay_us(15);
    ce_low();

    /* Poll IRQ pin (active LOW) or status register until TX_DS or MAX_RT */
    uint32_t deadline_ms = 1000;
    uint8_t status;
    do {
        vTaskDelay(pdMS_TO_TICKS(1));
        status = get_status(dev);
        deadline_ms--;
    } while (!(status & (STA_TX_DS | STA_MAX_RT)) && deadline_ms);

    /* Clear status flags */
    nrf24_write_reg(dev, REG_STATUS, STA_TX_DS | STA_MAX_RT);

    if (status & STA_MAX_RT) {
        uint8_t cmd = CMD_FLUSH_TX;
        spi_xfer(dev->spi, &cmd, NULL, 1);
        ESP_LOGW(TAG, "TX max retransmit reached");
        return ESP_ERR_TIMEOUT;
    }
    if (!deadline_ms) return ESP_ERR_TIMEOUT;

    return ESP_OK;
}

esp_err_t nrf24_receive(nrf24_t *dev, uint8_t *buf, uint8_t *len)
{
    uint8_t status = get_status(dev);

    if (!(status & STA_RX_DR)) {
        /* Also check FIFO directly in case IRQ flag was already cleared */
        uint8_t fifo = nrf24_read_reg(dev, REG_FIFO_STATUS);
        if (fifo & FIFO_RX_EMPTY) {
            return ESP_ERR_NOT_FOUND;
        }
    }

    /* Read dynamic or fixed payload width */
    uint8_t pw;
    uint8_t feat = nrf24_read_reg(dev, REG_FEATURE);
    if (feat & 0x04) {
        uint8_t tmp[2] = { CMD_R_RX_PL_WID, CMD_NOP };
        uint8_t rxb[2] = { 0 };
        spi_xfer(dev->spi, tmp, rxb, 2);
        pw = rxb[1];
    } else {
        pw = nrf24_read_reg(dev, REG_RX_PW_P0);
    }

    if (pw == 0 || pw > NRF24_MAX_PAYLOAD) {
        uint8_t cmd = CMD_FLUSH_RX;
        spi_xfer(dev->spi, &cmd, NULL, 1);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t tx_buf[pw + 1];
    uint8_t rx_buf[pw + 1];
    memset(tx_buf, CMD_NOP, sizeof(tx_buf));
    tx_buf[0] = CMD_R_RX_PAYLOAD;
    spi_xfer(dev->spi, tx_buf, rx_buf, pw + 1);

    memcpy(buf, &rx_buf[1], pw);
    *len = pw;

    /* Clear RX_DR flag */
    nrf24_write_reg(dev, REG_STATUS, STA_RX_DR);

    return ESP_OK;
}

esp_err_t nrf24_power_down(nrf24_t *dev)
{
    ce_low();
    nrf24_write_reg(dev, REG_CONFIG, CFG_EN_CRC | CFG_CRCO); /* PWR_UP=0 */
    return ESP_OK;
}

esp_err_t nrf24_power_up(nrf24_t *dev)
{
    uint8_t cfg = CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP;
    if (dev->prx_mode) cfg |= CFG_PRIM_RX;
    nrf24_write_reg(dev, REG_CONFIG, cfg);
    vTaskDelay(pdMS_TO_TICKS(5));
    if (dev->prx_mode) ce_high();
    return ESP_OK;
}
