#include <stdio.h>
#include "hardware/spi.h"
#include "hardware/structs/spi.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "spi_slave_config.h"

static bool g_spi_init_done = false;

void spi_slave_init(void) {
    if (g_spi_init_done) return;

    spi_init(SPI_SLAVE_INST, 12500u * 1000u);
    // spi_set_format(SPI_SLAVE_INST, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    hw_clear_bits(&spi_get_hw(SPI_SLAVE_INST)->cr1, SPI_SSPCR1_SSE_BITS);
    spi_set_slave(SPI_SLAVE_INST, true);
    spi_get_hw(SPI_SLAVE_INST)->cpsr = 2;
    hw_set_bits(&spi_get_hw(SPI_SLAVE_INST)->cr1, SPI_SSPCR1_SSE_BITS);

    gpio_set_function(SPI_TX_PIN,  GPIO_FUNC_SPI); // without MISO line
    gpio_set_function(SPI_RX_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_CS_PIN,  GPIO_FUNC_SPI);

    g_spi_init_done = true;
}

void spi_slave_deinit(void) {
    if (!g_spi_init_done) return;

    spi_deinit(SPI_SLAVE_INST);

    gpio_init(SPI_TX_PIN);
    gpio_init(SPI_RX_PIN);
    gpio_init(SPI_SCK_PIN);
    gpio_init(SPI_CS_PIN);

    g_spi_init_done = false;
}


// bool spi_central_send(const void *data, size_t len) {
//     if (!g_spi_init_done) return false;
//     spi_write_blocking(SPI_SLAVE_INST, (const uint8_t *)data, len);
//     return true;
// }

bool spi_central_ready(void) {
    return g_spi_init_done;
}
