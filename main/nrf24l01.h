#pragma once
#include "driver/spi_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#define NRF24_ADDR_WIDTH    5   /* bytes */
#define NRF24_MAX_PAYLOAD   32  /* bytes */

typedef enum {
    NRF24_DR_1MBPS   = 0,
    NRF24_DR_2MBPS   = 1,
    NRF24_DR_250KBPS = 2,
} nrf24_datarate_t;

typedef enum {
    NRF24_PA_M18DBM = 0,
    NRF24_PA_M12DBM = 1,
    NRF24_PA_M6DBM  = 2,
    NRF24_PA_0DBM   = 3,
} nrf24_pa_t;

typedef struct {
    spi_device_handle_t spi;
    bool                prx_mode;  /* true = receiver, false = transmitter */
} nrf24_t;

/*
 * Initialise the nRF24L01+ in PTX (transmitter) or PRX (receiver) mode.
 * channel  : 0–125 → RF frequency = 2400 + channel MHz
 * pipe_addr: 5-byte address, LSByte first (e.g. {0x89,0x67,0x45,0x23,0x01})
 * payload_w: fixed payload width for pipe 0 (1–32 bytes) — set 0 for dynamic
 */
esp_err_t nrf24_init_tx(nrf24_t *dev, uint8_t channel,
                         const uint8_t pipe_addr[NRF24_ADDR_WIDTH],
                         uint8_t payload_w, nrf24_datarate_t dr, nrf24_pa_t pa);

esp_err_t nrf24_init_rx(nrf24_t *dev, uint8_t channel,
                         const uint8_t pipe_addr[NRF24_ADDR_WIDTH],
                         uint8_t payload_w, nrf24_datarate_t dr, nrf24_pa_t pa);

/* Transmit `len` bytes (max 32). Blocks until ACK received or max retries.
 * Returns ESP_ERR_TIMEOUT on max-retransmit, ESP_OK on ACK. */
esp_err_t nrf24_transmit(nrf24_t *dev, const uint8_t *data, uint8_t len);

/* Poll for received payload. Returns ESP_ERR_NOT_FOUND when RX FIFO empty. */
esp_err_t nrf24_receive(nrf24_t *dev, uint8_t *buf, uint8_t *len);

/* Power down (lowest current consumption). */
esp_err_t nrf24_power_down(nrf24_t *dev);

/* Power up, restoring previous mode. */
esp_err_t nrf24_power_up(nrf24_t *dev);

uint8_t nrf24_read_reg(nrf24_t *dev, uint8_t reg);
void    nrf24_write_reg(nrf24_t *dev, uint8_t reg, uint8_t val);
