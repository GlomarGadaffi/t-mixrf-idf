#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "board.h"
#include "spi_bus.h"
#include "cc1101.h"
#include "nrf24l01.h"
#include "lr1121.h"
#include "st25r3916.h"

static const char *TAG = "main";

/* ── Demo task: CC1101 ────────────────────────────────────────────────── */
static void task_cc1101(void *arg)
{
    cc1101_t radio = { 0 };

    if (cc1101_init(&radio, CC1101_BAND_433, CC1101_MOD_OOK) != ESP_OK) {
        ESP_LOGE(TAG, "CC1101 init failed");
        vTaskDelete(NULL);
    }

    uint32_t tx_count = 0;
    for (;;) {
        /* Transmit */
        char msg[32];
        int mlen = snprintf(msg, sizeof(msg), "CC1101 #%lu", (unsigned long)tx_count++);
        esp_err_t err = cc1101_transmit(&radio, (uint8_t *)msg, (uint8_t)mlen);
        if (err == ESP_OK) {
            ESP_LOGI("cc1101", "TX ok: \"%s\"", msg);
        } else {
            ESP_LOGW("cc1101", "TX err: %s", esp_err_to_name(err));
        }

        /* Try receive for 200 ms */
        uint8_t buf[64];
        uint8_t rlen = 0;
        int8_t rssi = 0;
        err = cc1101_receive(&radio, buf, &rlen, &rssi, 200);
        if (err == ESP_OK) {
            buf[rlen] = '\0';
            ESP_LOGI("cc1101", "RX [%d bytes, %d dBm]: \"%s\"", rlen, rssi, buf);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ── Demo task: nRF24L01 ──────────────────────────────────────────────── */
static void task_nrf24(void *arg)
{
    nrf24_t radio = { 0 };

    /* 5-byte pipe address, LSByte first */
    const uint8_t addr[NRF24_ADDR_WIDTH] = { 0x89, 0x67, 0x45, 0x23, 0x01 };

    if (nrf24_init_tx(&radio, 76, addr, 32, NRF24_DR_1MBPS, NRF24_PA_0DBM) != ESP_OK) {
        ESP_LOGE(TAG, "nRF24 init failed");
        vTaskDelete(NULL);
    }

    uint32_t count = 0;
    for (;;) {
        char msg[32];
        uint8_t len = (uint8_t)snprintf(msg, sizeof(msg), "NRF24 #%lu", (unsigned long)count++);

        esp_err_t err = nrf24_transmit(&radio, (uint8_t *)msg, len);
        if (err == ESP_OK) {
            ESP_LOGI("nrf24", "TX ACK: \"%s\"", msg);
        } else {
            ESP_LOGW("nrf24", "TX no-ACK: %s", esp_err_to_name(err));
        }

        /* Check RX buffer */
        uint8_t buf[NRF24_MAX_PAYLOAD];
        uint8_t rlen = 0;
        if (nrf24_receive(&radio, buf, &rlen) == ESP_OK) {
            buf[rlen] = '\0';
            ESP_LOGI("nrf24", "RX [%d bytes]: \"%s\"", rlen, buf);
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* ── Demo task: LR1121 ────────────────────────────────────────────────── */
static void task_lr1121(void *arg)
{
    lr1121_t radio = { 0 };

    /* 915 MHz (FCC Part 15, 902-928 MHz ISM), SF10, BW 125 kHz, CR 4/6, +14 dBm */
    if (lr1121_init(&radio, 915000000UL,
                    10, LR1121_BW_125, LR1121_CR_4_6, 14) != ESP_OK) {
        ESP_LOGE(TAG, "LR1121 init failed");
        vTaskDelete(NULL);
    }

    uint32_t count = 0;
    for (;;) {
        char msg[64];
        uint8_t len = (uint8_t)snprintf(msg, sizeof(msg),
                                         "LR1121 LoRa #%lu", (unsigned long)count++);

        esp_err_t err = lr1121_transmit(&radio, (uint8_t *)msg, len, 5000);
        if (err == ESP_OK) {
            ESP_LOGI("lr1121", "TX done: \"%s\"", msg);
        } else {
            ESP_LOGW("lr1121", "TX err: %s", esp_err_to_name(err));
        }

        /* RX window: listen for 2 seconds */
        uint8_t buf[256];
        uint8_t rlen = 0;
        int8_t rssi = 0, snr = 0;
        err = lr1121_receive(&radio, buf, &rlen, &rssi, &snr, 2000);
        if (err == ESP_OK) {
            buf[rlen] = '\0';
            ESP_LOGI("lr1121", "RX [%d B, %d dBm, SNR %d]: \"%s\"",
                     rlen, rssi, snr, buf);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ── Demo task: ST25R3916 NFC ─────────────────────────────────────────── */
static void task_nfc(void *arg)
{
    st25r3916_t reader = { 0 };

    if (st25r3916_init(&reader) != ESP_OK) {
        ESP_LOGE(TAG, "ST25R3916 init failed");
        vTaskDelete(NULL);
    }

    for (;;) {
        uint8_t uid[10];
        uint8_t uid_len = 0;

        esp_err_t err = st25r3916_detect_iso14443a(&reader, uid, &uid_len);
        if (err == ESP_OK) {
            char uid_str[32] = { 0 };
            for (int i = 0; i < uid_len; i++) {
                char byte[4];
                snprintf(byte, sizeof(byte), "%s%02X", i ? ":" : "", uid[i]);
                strncat(uid_str, byte, sizeof(uid_str) - strlen(uid_str) - 1);
            }
            ESP_LOGI("nfc", "Card UID: %s", uid_str);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "T-MixRF ESP-IDF — starting");

    /* Shared SPI bus must be initialised before any module task starts. */
    ESP_ERROR_CHECK(spi_bus_init());

    /* Each radio runs in its own task so they can operate concurrently.
     * Stack sizes are generous — tune down if RAM is tight. */
    xTaskCreate(task_cc1101, "cc1101", 4096, NULL, 5, NULL);
    xTaskCreate(task_nrf24,  "nrf24",  4096, NULL, 5, NULL);
    xTaskCreate(task_lr1121, "lr1121", 4096, NULL, 5, NULL);
    xTaskCreate(task_nfc,    "nfc",    4096, NULL, 5, NULL);
}
