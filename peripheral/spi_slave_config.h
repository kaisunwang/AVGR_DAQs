#ifndef SPI_SLAVE_CONFIG_H
#define SPI_SLAVE_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// SPI1 pin mapping (peripheral is slave; central drives SCK and CS)
#define SPI_SLAVE_INST      spi1
#define SPI_TX_PIN          15u   // MISO: slave TX → central RX
#define SPI_RX_PIN          12u   // MOSI: central TX → slave RX
#define SPI_SCK_PIN         14u   // SCK: clock driven by central
#define SPI_CS_PIN          13u   // CSn: chip select driven by central

// Handshake pin (input from central: HIGH = "go, send data")
#define ACK_PIN             26

// Initialise SPI1 hardware + GPIO functions. Configures ACK_PIN as input.
void spi_slave_init(void);

// Tear down SPI1 hardware; release SPI GPIO pins back to plain input.
void spi_slave_deinit(void);

// bool spi_central_send(const void *data, size_t len);

// Returns true if spi_slave_init() has been called (and deinit not yet called).
bool spi_central_ready(void);

#endif
