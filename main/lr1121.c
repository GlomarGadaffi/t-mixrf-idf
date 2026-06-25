#include "lr1121.h"
#include "board.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "lr1121";

/* ── Opcodes — verified against RadioLib LR11x0_commands.h ───────────── */

/* System group (0x01xx) */
#define CMD_GET_STATUS          0x0100
#define CMD_GET_VERSION         0x0101
#define CMD_WRITE_REG_MEM       0x0105
#define CMD_READ_REG_MEM        0x0106
#define CMD_WRITE_BUFFER        0x0109
#define CMD_READ_BUFFER         0x010A
#define CMD_CLEAR_RX_BUFFER     0x010B
#define CMD_GET_ERRORS          0x010D
#define CMD_CLEAR_ERRORS        0x010E
#define CMD_CALIBRATE           0x010F
#define CMD_SET_REG_MODE        0x0110
#define CMD_CALIB_IMAGE         0x0111
#define CMD_SET_DIO_AS_RF_SWITCH 0x0112
#define CMD_SET_DIO_IRQ_PARAMS  0x0113
#define CMD_CLEAR_IRQ           0x0114
#define CMD_SET_TCXO_MODE       0x0117
#define CMD_REBOOT              0x0118
#define CMD_SET_SLEEP           0x011B
#define CMD_SET_STANDBY         0x011C
#define CMD_SET_FS              0x011D

/* Radio group (0x02xx) */
#define CMD_GET_PACKET_TYPE     0x0202
#define CMD_GET_RX_BUFFER_STATUS 0x0203
#define CMD_GET_PACKET_STATUS   0x0204
#define CMD_SET_RX              0x0209
#define CMD_SET_TX              0x020A
#define CMD_SET_RF_FREQUENCY    0x020B
#define CMD_SET_PACKET_TYPE     0x020E
#define CMD_SET_MODULATION_PARAMS 0x020F
#define CMD_SET_PACKET_PARAMS   0x0210
#define CMD_SET_TX_PARAMS       0x0211
#define CMD_SET_PA_CONFIG       0x0215
#define CMD_STOP_TIMEOUT_ON_PREAMBLE 0x0217
#define CMD_SET_LORA_SYNC_WORD  0x022B
#define CMD_SET_RX_BOOSTED      0x0227

/* ── Packet type ──────────────────────────────────────────────────────── */
#define PKT_TYPE_GFSK  0x00
#define PKT_TYPE_LORA  0x01

/* ── Standby modes ────────────────────────────────────────────────────── */
#define STANDBY_RC   0x00   /* RC oscillator ~32 kHz, ~100 nA */
#define STANDBY_XOSC 0x01   /* 32 MHz XOSC/TCXO, higher power */

/* ── PA configuration ─────────────────────────────────────────────────── */
/* PA selection: 0x00 = LP (low-power, sub-GHz <15 dBm),
 *               0x01 = HP (high-power, sub-GHz up to 22 dBm),
 *               0x02 = HF (2.4 GHz) */
#define PA_SEL_LP  0x00
#define PA_SEL_HP  0x01
#define PA_SEL_HF  0x02
/* Supply: 0x00 = VBAT, 0x01 = VREG */
#define PA_SUPPLY_VREG 0x00
#define PA_SUPPLY_VBAT 0x01

/* ── TCXO voltage byte ────────────────────────────────────────────────── */
/* 0x00=1.6V 0x01=1.7V 0x02=1.8V 0x03=2.2V
 * 0x04=2.4V 0x05=2.7V 0x06=3.0V 0x07=3.3V  (see LR1121 datasheet §3.6) */
#define TCXO_3_3V  0x07

/* ── IRQ mask bits ────────────────────────────────────────────────────── */
#define IRQ_TX_DONE         (1UL << 2)
#define IRQ_RX_DONE         (1UL << 3)
#define IRQ_HEADER_ERR      (1UL << 9)
#define IRQ_CRC_ERR         (1UL << 10)
#define IRQ_TIMEOUT         (1UL << 0)
#define IRQ_ALL             0xFFFFFFFF

/* ── LR1121 crystal / TCXO frequency ────────────────────────────────── */
#define LR1121_CRYSTAL_HZ   32000000UL
#define LR1121_DIV_EXP      25          /* freq reg = f_Hz * 2^25 / 32e6 */

/* ── SPI two-phase protocol ──────────────────────────────────────────── */
/*
 * LR1121 SPI transaction model (BUSY-gated):
 *
 * Phase A — write:  CS↓ | opcode[2] | params[N] | CS↑  →  wait BUSY=LOW
 * Phase B — read :  CS↓ | NOP[1] | response[M] | CS↑    (read commands only)
 *
 * The BUSY pin is held HIGH by the chip while it processes a command.
 * We must not start a new transaction until BUSY returns LOW.
 */

static void wait_busy(void)
{
    /* Typical busy duration: <1 ms for most commands. Cap at 100 ms. */
    uint32_t deadline = 1000;
    while (gpio_get_level(PIN_LR1121_BUSY) && --deadline) {
        esp_rom_delay_us(100);
    }
    if (!deadline) {
        ESP_LOGE("lr1121", "BUSY timeout");
    }
}

/* Send opcode + write bytes; poll BUSY before returning. */
static void cmd_write(spi_device_handle_t spi,
                      uint16_t opcode, const uint8_t *params, size_t n)
{
    size_t total = 2 + n;
    uint8_t tx[total];
    tx[0] = (uint8_t)(opcode >> 8);
    tx[1] = (uint8_t)(opcode & 0xFF);
    if (n) memcpy(&tx[2], params, n);

    spi_transaction_t t = {
        .length    = total * 8,
        .tx_buffer = tx,
        .rx_buffer = NULL,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
    wait_busy();
}

/* Send opcode (no params), then read back `m` bytes in a second CS pulse. */
static void cmd_read(spi_device_handle_t spi,
                     uint16_t opcode, uint8_t *dst, size_t m)
{
    /* Phase A */
    uint8_t tx_a[2] = { (uint8_t)(opcode >> 8), (uint8_t)(opcode & 0xFF) };
    spi_transaction_t ta = {
        .length    = 16,
        .tx_buffer = tx_a,
        .rx_buffer = NULL,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &ta));
    wait_busy();

    /* Phase B: send 1 NOP byte (status), then read m data bytes */
    size_t rb_len = 1 + m;
    uint8_t tx_b[rb_len];
    uint8_t rx_b[rb_len];
    memset(tx_b, 0x00, rb_len);

    spi_transaction_t tb = {
        .length    = rb_len * 8,
        .tx_buffer = tx_b,
        .rx_buffer = rx_b,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &tb));
    /* rx_b[0] is the chip status byte; actual data starts at rx_b[1] */
    memcpy(dst, &rx_b[1], m);
}

/* ── GPIO init helpers ────────────────────────────────────────────────── */
static void gpio_out(int pin, int initial)
{
    gpio_config_t c = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&c));
    gpio_set_level(pin, initial);
}

static void gpio_in(int pin)
{
    gpio_config_t c = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&c));
}

/* ── Frequency calculation ────────────────────────────────────────────── */
static void freq_to_bytes(uint32_t freq_hz, uint8_t out[4])
{
    /* freq_reg = freq_hz * 2^25 / 32e6 — use 64-bit to avoid overflow */
    uint64_t reg = ((uint64_t)freq_hz << 25) / LR1121_CRYSTAL_HZ;
    out[0] = (uint8_t)(reg >> 24);
    out[1] = (uint8_t)(reg >> 16);
    out[2] = (uint8_t)(reg >>  8);
    out[3] = (uint8_t)(reg);
}

/* ── Public API ──────────────────────────────────────────────────────── */

esp_err_t lr1121_reset(lr1121_t *dev)
{
    gpio_set_level(PIN_LR1121_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(PIN_LR1121_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    wait_busy();
    return ESP_OK;
}

esp_err_t lr1121_get_version(lr1121_t *dev,
                              uint8_t *hw, uint8_t *use_case,
                              uint8_t *fw_maj, uint8_t *fw_min)
{
    uint8_t resp[4] = { 0 };
    cmd_read(dev->spi, CMD_GET_VERSION, resp, 4);
    if (hw)       *hw       = resp[0];
    if (use_case) *use_case = resp[1];
    if (fw_maj)   *fw_maj   = resp[2];
    if (fw_min)   *fw_min   = resp[3];
    return ESP_OK;
}

esp_err_t lr1121_standby(lr1121_t *dev)
{
    uint8_t p = STANDBY_RC;
    cmd_write(dev->spi, CMD_SET_STANDBY, &p, 1);
    return ESP_OK;
}

esp_err_t lr1121_init(lr1121_t *dev, uint32_t freq_hz,
                       uint8_t sf, lr1121_bw_t bw, lr1121_cr_t cr,
                       int8_t power_dbm)
{
    dev->freq_hz   = freq_hz;
    dev->sf        = sf;
    dev->bw        = bw;
    dev->cr        = cr;
    dev->power_dbm = power_dbm;

    /* GPIO setup */
    gpio_out(PIN_LR1121_RST,    1);
    gpio_out(PIN_LR1121_SWITCH, 1);
    gpio_in (PIN_LR1121_BUSY);
    gpio_in (PIN_LR1121_DIO9);

    /* SPI device — LR1121 Mode 0, up to 16 MHz */
    spi_device_interface_config_t dcfg = {
        .clock_speed_hz = 16 * 1000 * 1000,
        .mode           = 0,
        .spics_io_num   = PIN_LR1121_CS,
        .queue_size     = 4,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dcfg, &dev->spi));

    /* Hardware reset */
    lr1121_reset(dev);

    /* Verify firmware version */
    uint8_t hw, uc, maj, min;
    lr1121_get_version(dev, &hw, &uc, &maj, &min);
    ESP_LOGI(TAG, "LR1121 hw=0x%02X use_case=0x%02X fw=%d.%d", hw, uc, maj, min);
    if (hw == 0x00 && maj == 0x00) {
        ESP_LOGE(TAG, "Invalid version — check wiring or PSRAM conflict on GPIO33-37");
        return ESP_ERR_NOT_FOUND;
    }

    /* Standby-XOSC so we can configure TCXO */
    uint8_t sb_xosc = STANDBY_XOSC;
    cmd_write(dev->spi, CMD_SET_STANDBY, &sb_xosc, 1);

    /* TCXO mode: 3.3 V, ~5 ms stabilisation (delay = 164 × 30.5 µs) */
    {
        uint8_t p[4] = { TCXO_3_3V, 0x00, 0x00, 0xA4 }; /* delay=164 */
        cmd_write(dev->spi, CMD_SET_TCXO_MODE, p, 4);
    }

    /* Calibrate: all blocks (0xFF = all calibration items) */
    {
        uint8_t p = 0xFF;
        cmd_write(dev->spi, CMD_CALIBRATE, &p, 1);
        vTaskDelay(pdMS_TO_TICKS(50)); /* calibration takes up to 50 ms */
        wait_busy();
    }

    /* Image calibration for the target band
     * Sub-GHz 863–928 MHz: freq1=0xD7, freq2=0xDB
     * 2.4 GHz:              freq1=0xE1, freq2=0xE9  */
    {
        uint8_t p[2];
        if (freq_hz < 1000000000UL) {
            p[0] = 0xD7; p[1] = 0xDB; /* 863–928 MHz */
        } else {
            p[0] = 0xE1; p[1] = 0xE9; /* 2400–2500 MHz */
        }
        cmd_write(dev->spi, CMD_CALIB_IMAGE, p, 2);
    }

    /* Standby-RC for configuration */
    lr1121_standby(dev);

    /* RF switch GPIO (antenna path selector, driven by host ESP32 GPIO7) */
    gpio_set_level(PIN_LR1121_SWITCH,
                   (freq_hz < 1000000000UL) ? 1 : 0); /* HIGH=sub-GHz */

    /* Packet type: LoRa */
    {
        uint8_t p = PKT_TYPE_LORA;
        cmd_write(dev->spi, CMD_SET_PACKET_TYPE, &p, 1);
    }

    /* RF frequency */
    {
        uint8_t p[4];
        freq_to_bytes(freq_hz, p);
        cmd_write(dev->spi, CMD_SET_RF_FREQUENCY, p, 4);
    }

    /* PA configuration
     * HP PA (sub-GHz high-power): pa_sel=1, supply=VREG, duty_cycle=0x04,
     *                              hp_sel=0x07 (max gain stages)
     * HF PA (2.4 GHz):            pa_sel=2, supply=VREG, duty_cycle=0x04,
     *                              hp_sel=0x00
     * LP PA (sub-GHz low-power):  pa_sel=0, supply=VREG, duty_cycle=0x04,
     *                              hp_sel=0x00
     */
    {
        uint8_t pa_sel, hp_sel;
        if (freq_hz >= 2000000000UL) {
            pa_sel = PA_SEL_HF; hp_sel = 0x00;
        } else if (power_dbm > 14) {
            pa_sel = PA_SEL_HP; hp_sel = 0x07;
        } else {
            pa_sel = PA_SEL_LP; hp_sel = 0x00;
        }
        uint8_t p[4] = { pa_sel, PA_SUPPLY_VREG, 0x04, hp_sel };
        cmd_write(dev->spi, CMD_SET_PA_CONFIG, p, 4);
    }

    /* TX parameters: power (signed dBm), ramp time 200 µs (0x04) */
    {
        uint8_t p[2] = { (uint8_t)(int8_t)power_dbm, 0x04 };
        cmd_write(dev->spi, CMD_SET_TX_PARAMS, p, 2);
    }

    /* LoRa modulation parameters: SF, BW, CR, LDRO
     * LDRO (low data rate optimise): auto-enable when SF≥11 and BW<500 kHz */
    {
        uint8_t ldro = (sf >= 11 && bw < LR1121_BW_500) ? 0x01 : 0x00;
        uint8_t p[4] = { sf, (uint8_t)bw, (uint8_t)cr, ldro };
        cmd_write(dev->spi, CMD_SET_MODULATION_PARAMS, p, 4);
    }

    /* LoRa packet parameters:
     * preamble length (2 bytes MSB first), header type (explicit=0),
     * payload length, CRC (on=1), IQ invert (off=0) */
    {
        uint8_t p[6] = { 0x00, 0x0C,  /* preamble = 12 symbols */
                         0x00,         /* explicit header */
                         0xFF,         /* max payload */
                         0x01,         /* CRC on */
                         0x00 };       /* IQ not inverted */
        cmd_write(dev->spi, CMD_SET_PACKET_PARAMS, p, 6);
    }

    /* LoRa sync word: 0x12 = private network, 0x34 = LoRaWAN public */
    {
        uint8_t p[2] = { 0x00, 0x12 };
        cmd_write(dev->spi, CMD_SET_LORA_SYNC_WORD, p, 2);
    }

    /* Clear any pending errors / IRQ */
    cmd_write(dev->spi, CMD_CLEAR_ERRORS, NULL, 0);
    {
        uint8_t p[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
        cmd_write(dev->spi, CMD_CLEAR_IRQ, p, 4);
    }

    /* Set DIO IRQ params: DIO9 triggers on TX_DONE | RX_DONE | TIMEOUT | CRC_ERR */
    {
        uint32_t mask = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_TIMEOUT |
                        IRQ_HEADER_ERR | IRQ_CRC_ERR;
        uint8_t p[8] = {
            (uint8_t)(mask >> 24), (uint8_t)(mask >> 16),
            (uint8_t)(mask >>  8), (uint8_t)(mask),
            0x00, 0x00, 0x00, 0x00  /* DIO other: none */
        };
        cmd_write(dev->spi, CMD_SET_DIO_IRQ_PARAMS, p, 8);
    }

    ESP_LOGI(TAG, "LR1121 ready — freq=%lu Hz SF=%d CR=%d",
             (unsigned long)freq_hz, sf, cr);
    return ESP_OK;
}

esp_err_t lr1121_transmit(lr1121_t *dev, const uint8_t *data, uint8_t len,
                           uint32_t timeout_ms)
{
    /* Write payload to chip buffer at offset 0 */
    {
        uint8_t params[len + 1];
        params[0] = 0x00; /* buffer offset */
        memcpy(&params[1], data, len);
        cmd_write(dev->spi, CMD_WRITE_BUFFER, params, len + 1);
    }

    /* Update packet params with actual payload length */
    {
        uint8_t p[6] = { 0x00, 0x0C, 0x00, len, 0x01, 0x00 };
        cmd_write(dev->spi, CMD_SET_PACKET_PARAMS, p, 6);
    }

    /* Clear IRQ, set antenna switch, start TX (timeout in ms * 32 = tick units)
     * Timeout value 0x000000 = no timeout (single TX) */
    {
        uint8_t clr[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
        cmd_write(dev->spi, CMD_CLEAR_IRQ, clr, 4);
    }

    gpio_set_level(PIN_LR1121_SWITCH,
                   (dev->freq_hz < 1000000000UL) ? 1 : 0);

    {
        uint8_t p[3] = { 0x00, 0x00, 0x00 }; /* no timeout */
        cmd_write(dev->spi, CMD_SET_TX, p, 3);
    }

    /* Poll DIO9 (active HIGH when IRQ fires) until TX_DONE or timeout */
    uint32_t deadline = timeout_ms;
    while (!gpio_get_level(PIN_LR1121_DIO9) && deadline) {
        vTaskDelay(pdMS_TO_TICKS(1));
        deadline--;
    }

    if (deadline == 0) {
        lr1121_standby(dev);
        return ESP_ERR_TIMEOUT;
    }

    /* Clear all IRQ flags */
    uint8_t irq_clr[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
    cmd_write(dev->spi, CMD_CLEAR_IRQ, irq_clr, 4);

    lr1121_standby(dev);
    return ESP_OK;
}

esp_err_t lr1121_receive(lr1121_t *dev, uint8_t *buf, uint8_t *len,
                          int8_t *rssi, int8_t *snr, uint32_t timeout_ms)
{
    /* Clear IRQ, set antenna switch */
    {
        uint8_t p[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
        cmd_write(dev->spi, CMD_CLEAR_IRQ, p, 4);
    }

    gpio_set_level(PIN_LR1121_SWITCH,
                   (dev->freq_hz < 1000000000UL) ? 1 : 0);

    /* Start RX — timeout in units of 15.625 µs; 0x000000 = continuous */
    {
        uint8_t p[3] = { 0x00, 0x00, 0x00 };
        cmd_write(dev->spi, CMD_SET_RX, p, 3);
    }

    /* Poll DIO9 */
    uint32_t deadline = timeout_ms;
    while (!gpio_get_level(PIN_LR1121_DIO9) && deadline) {
        vTaskDelay(pdMS_TO_TICKS(1));
        deadline--;
    }

    lr1121_standby(dev);

    if (deadline == 0) return ESP_ERR_TIMEOUT;

    /* Read RX buffer status: payload size, offset */
    uint8_t rx_stat[2] = { 0 };
    cmd_read(dev->spi, CMD_GET_RX_BUFFER_STATUS, rx_stat, 2);
    uint8_t payload_len    = rx_stat[0];
    uint8_t buffer_offset  = rx_stat[1];

    if (payload_len == 0 || payload_len > 255) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Read packet status: RSSI (1 byte, unsigned), SNR (1 byte, signed) */
    {
        uint8_t pkt_stat[3] = { 0 };
        cmd_read(dev->spi, CMD_GET_PACKET_STATUS, pkt_stat, 3);
        if (rssi) *rssi = -(int8_t)(pkt_stat[0] >> 1); /* RSSI = -RssiPkt/2 */
        if (snr)  *snr  = (int8_t)pkt_stat[1] / 4;
    }

    /* Read payload from buffer */
    {
        uint8_t p[2] = { buffer_offset, payload_len };
        uint8_t resp[payload_len];
        /* Phase A: send opcode + {offset, length} */
        {
            uint8_t tx_a[4] = {
                (uint8_t)(CMD_READ_BUFFER >> 8),
                (uint8_t)(CMD_READ_BUFFER & 0xFF),
                buffer_offset, payload_len
            };
            spi_transaction_t ta = {
                .length    = 32,
                .tx_buffer = tx_a,
                .rx_buffer = NULL,
            };
            ESP_ERROR_CHECK(spi_device_polling_transmit(dev->spi, &ta));
            wait_busy();
        }
        /* Phase B: NOP + payload bytes */
        {
            uint8_t tx_b[payload_len + 1];
            uint8_t rx_b[payload_len + 1];
            memset(tx_b, 0x00, sizeof(tx_b));
            spi_transaction_t tb = {
                .length    = (payload_len + 1) * 8,
                .tx_buffer = tx_b,
                .rx_buffer = rx_b,
            };
            ESP_ERROR_CHECK(spi_device_polling_transmit(dev->spi, &tb));
            memcpy(buf, &rx_b[1], payload_len);
        }
        *len = payload_len;
    }

    /* Clear IRQ */
    {
        uint8_t p[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
        cmd_write(dev->spi, CMD_CLEAR_IRQ, p, 4);
    }

    return ESP_OK;
}
