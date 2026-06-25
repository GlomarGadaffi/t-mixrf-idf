#include "spi_bus.h"
#include "board.h"
#include "esp_log.h"

static const char *TAG = "spi_bus";

esp_err_t spi_bus_init(void)
{
    spi_bus_config_t cfg = {
        .mosi_io_num     = PIN_SPI_MOSI,
        .miso_io_num     = PIN_SPI_MISO,
        .sclk_io_num     = PIN_SPI_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 256,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPI2 initialized (SCK=%d MOSI=%d MISO=%d)",
                 PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_MISO);
    }
    return ret;
}
