#include "st25r3916.h"
#include "board.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "st25r3916";

/*
 * ST25R3916 SPI protocol (SPI Mode 1: CPOL=0, CPHA=1):
 *
 *   Write register : 0x00|addr , data
 *   Read  register : 0x40|addr , [dummy] → data
 *   Write FIFO     : 0x80       , data[N]
 *   Read  FIFO     : 0xBF       , [dummy] → data[N]
 *   Direct command : 0xC0|cmd
 *
 * NOTE: If the hardware does not respond try SPI Mode 3 (CPOL=1, CPHA=1).
 * The ST25R3916 datasheet describes "falling-edge output / rising-edge input"
 * which corresponds to Mode 1.  Some boards use Mode 3 — check your schematic.
 */
#define ST25_WRITE_REG  0x00
#define ST25_READ_REG   0x40
#define ST25_WRITE_FIFO 0x80
#define ST25_READ_FIFO  0xBF
#define ST25_DIRECT_CMD 0xC0

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

uint8_t st25r3916_read_reg(st25r3916_t *dev, uint8_t addr)
{
    uint8_t tx[2] = { ST25_READ_REG | (addr & 0x3F), 0x00 };
    uint8_t rx[2] = { 0 };
    spi_xfer(dev->spi, tx, rx, 2);
    return rx[1];
}

void st25r3916_write_reg(st25r3916_t *dev, uint8_t addr, uint8_t val)
{
    uint8_t tx[2] = { ST25_WRITE_REG | (addr & 0x3F), val };
    spi_xfer(dev->spi, tx, NULL, 2);
}

void st25r3916_cmd(st25r3916_t *dev, uint8_t cmd)
{
    uint8_t tx = ST25_DIRECT_CMD | (cmd & 0x3F);
    spi_xfer(dev->spi, &tx, NULL, 1);
}

static void fifo_write(st25r3916_t *dev, const uint8_t *data, uint8_t len)
{
    uint8_t tx[len + 1];
    tx[0] = ST25_WRITE_FIFO;
    memcpy(&tx[1], data, len);
    spi_xfer(dev->spi, tx, NULL, len + 1);
}

static void fifo_read(st25r3916_t *dev, uint8_t *dst, uint8_t len)
{
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];
    memset(tx, 0x00, sizeof(tx));
    tx[0] = ST25_READ_FIFO;
    spi_xfer(dev->spi, tx, rx, len + 1);
    memcpy(dst, &rx[1], len);
}

esp_err_t st25r3916_init(st25r3916_t *dev)
{
    /* EN pin: power on analog front-end */
    gpio_config_t io_en = {
        .pin_bit_mask = (1ULL << PIN_NFC_EN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_en));
    gpio_set_level(PIN_NFC_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(PIN_NFC_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* SPI device — ST25R3916 Mode 1, max 6 MHz */
    spi_device_interface_config_t dcfg = {
        .clock_speed_hz = 4 * 1000 * 1000,
        .mode           = 1,               /* CPOL=0, CPHA=1 */
        .spics_io_num   = PIN_NFC_CS,
        .queue_size     = 4,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dcfg, &dev->spi));

    /* Soft reset via direct command */
    st25r3916_cmd(dev, 0x01); /* SET_DEFAULT (0xC1 on wire) */
    vTaskDelay(pdMS_TO_TICKS(3));

    /* Check IC identity — should return 0x09 (ST25R3916) or 0x0B (ST25R3916B) */
    uint8_t id = st25r3916_read_reg(dev, ST25_REG_IC_IDENT);
    ESP_LOGI(TAG, "ST25R3916 IC_IDENT=0x%02X", id);
    if (id != 0x09 && id != 0x0B) {
        ESP_LOGW(TAG, "Unexpected IC_IDENT (expected 0x09 or 0x0B) — check SPI mode");
    }

    /* Adjust regulators */
    st25r3916_cmd(dev, 0x0B); /* ADJUST_REGULATORS */
    vTaskDelay(pdMS_TO_TICKS(5));

    /* IO configuration: single-ended antenna, OOK modulation, no pull */
    st25r3916_write_reg(dev, ST25_REG_IO_CONF1, 0x00);
    st25r3916_write_reg(dev, ST25_REG_IO_CONF2, 0x00);

    /* Mode: ISO14443A, 106 kbps */
    st25r3916_write_reg(dev, ST25_REG_MODE_DEF, ST25_MODE_ISO14443A);

    /* Bit rate: 106 kbps TX and RX */
    st25r3916_write_reg(dev, ST25_REG_BIT_RATE, 0x00);

    /* ISO14443A/NFC: NFC-A enable, TX2 drive off, 100% modulation */
    st25r3916_write_reg(dev, ST25_REG_NFC_IP1, 0x00);

    /* Enable analog/digital parts */
    st25r3916_write_reg(dev, ST25_REG_OP_CONTROL,
                        ST25_OP_EN | ST25_OP_RX_EN | ST25_OP_TX_EN);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Calibrate antenna modulation */
    st25r3916_cmd(dev, 0x0C); /* CALIBRATE_MODULATION */
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "ST25R3916 ready (ISO14443A 106kbps)");
    return ESP_OK;
}

/*
 * Minimal ISO14443-A REQA + anti-collision (for UID retrieval).
 *
 * This is a simplified polling implementation using direct FIFO access.
 * For production NFC use, integrate ST's RFAL (RF Abstraction Layer) from
 * https://github.com/stm32duino/NFC-RFAL  — it handles all protocol details.
 */
esp_err_t st25r3916_detect_iso14443a(st25r3916_t *dev,
                                      uint8_t *uid, uint8_t *uid_len)
{
    *uid_len = 0;

    /* Clear FIFO, stop any previous activity */
    st25r3916_cmd(dev, 0x02); /* STOP */
    vTaskDelay(pdMS_TO_TICKS(2));

    /* Send REQA (0x26) as a 7-bit frame — configure for short frame first */
    /* Short frame mode: set bit 4 of NFC_IP1 (NFCIP1_SEND_SDD bit varies
     * by silicon rev; consult table 56 in the ST25R3916 datasheet).
     * For simplicity we use the TX command with CRC disabled. */

    /* Load REQA byte into FIFO */
    uint8_t reqa = 0x26;
    fifo_write(dev, &reqa, 1);

    /* Transmit without CRC (short frame) */
    st25r3916_cmd(dev, 0x04); /* TRANSMIT_WITHOUT_CRC */
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Wait for receive (poll FIFO status) — simplified: check main interrupt
     * register bit for end-of-receive. Real code should poll INT registers. */
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Read ATQA response (2 bytes) */
    uint8_t atqa[2] = { 0 };
    fifo_read(dev, atqa, 2);

    if (atqa[0] == 0x00 && atqa[1] == 0x00) {
        return ESP_ERR_NOT_FOUND; /* no card */
    }
    ESP_LOGI(TAG, "ATQA=0x%02X%02X", atqa[1], atqa[0]);

    /* Anti-collision loop (cascade level 1: SEL=0x93, NVB=0x20) */
    uint8_t anticoll[2] = { 0x93, 0x20 };
    fifo_write(dev, anticoll, 2);
    st25r3916_cmd(dev, 0x04);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Read 5-byte cascade-1 UID (CT + UID[0..2] + BCC) */
    uint8_t uid5[5] = { 0 };
    fifo_read(dev, uid5, 5);

    /* BCC validation: uid5[0]^uid5[1]^uid5[2]^uid5[3] should equal uid5[4] */
    uint8_t bcc = uid5[0] ^ uid5[1] ^ uid5[2] ^ uid5[3];
    if (bcc != uid5[4]) {
        ESP_LOGW(TAG, "BCC mismatch (got 0x%02X expected 0x%02X)", uid5[4], bcc);
        return ESP_ERR_INVALID_CRC;
    }

    memcpy(uid, uid5, 4);
    *uid_len = 4;

    ESP_LOGI(TAG, "UID: %02X:%02X:%02X:%02X", uid[0], uid[1], uid[2], uid[3]);
    return ESP_OK;
}
