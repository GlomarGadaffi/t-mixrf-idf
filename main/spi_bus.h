#pragma once
#include "driver/spi_master.h"
#include "esp_err.h"

/*
 * All four radio modules share SPI2_HOST (SCK=17, MOSI=15, MISO=16).
 * Call spi_bus_init() once from app_main before touching any module.
 */
esp_err_t spi_bus_init(void);
