/**
 * PIO capture helpers: bit-packing utilities and state machine setup
 * for 8-bit parallel sampling with ADC clock sideset.
 * // functions stolen from old version
 */

#include "capture.h"

#include "hardware/gpio.h"
#include "hardware/pio.h"

/* Number of bits stored per 32-bit PIO word for a given pin_count.
 * PIO packs bits tightly with no padding inside each word. */
uint bits_packed_per_word(uint pin_count) {
    const uint SHIFT_REG_WIDTH = 32;
    return SHIFT_REG_WIDTH - (SHIFT_REG_WIDTH % pin_count);
}

/* Total 32-bit words needed to hold 'samples' logical samples. */
uint32_t words_for_samples(uint32_t samples, uint32_t pin_count) {
    uint32_t bits_per_word = bits_packed_per_word(pin_count);
    uint32_t total_bits    = samples * pin_count;
    return (total_bits + bits_per_word - 1) / bits_per_word;
}

/* Build and load a 2-instruction PIO program:
 *   cycle 0: IN  pins, 8   [sideset CLK=1]   — sample + clock high
 *   cycle 1: NOP           [sideset CLK=0]   — clock low
 * Returns the program offset in PIO instruction memory. */
uint clk_sample_program_load(PIO pio) {
    uint16_t prog[2];
    prog[0] = (uint16_t)(pio_encode_in(pio_pins, 8) | pio_encode_sideset(1, 1));
    prog[1] = (uint16_t)(pio_encode_nop()           | pio_encode_sideset(1, 0));

    struct pio_program p = {
        .instructions = (const uint16_t *)prog,
        .length       = 2,
        .origin       = -1,
    };
    return pio_add_program(pio, &p);
}

/* Configure and start the capture state machine.
 * clk_pin     : GPIO driven as ADC clock (sideset output)
 * data_pin_base : first of CAPTURE_PIN_COUNT consecutive input GPIOs
 * clkdiv      : PIO clock divider (sys_clk / (2 * target_sample_hz)) */
void clk_sample_sm_init(PIO pio, uint sm, uint offset,
                         uint clk_pin, uint data_pin_base, float clkdiv) {
    pio_sm_config c = pio_get_default_sm_config();

    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, clk_pin);
    sm_config_set_in_pins(&c, data_pin_base);
    sm_config_set_in_shift(&c, true, true, 32);   // shift right, autopush at 32 bits
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_wrap(&c, offset, offset + 1);
    sm_config_set_clkdiv(&c, clkdiv);

    gpio_set_function(clk_pin, GPIO_FUNC_PIO0);
    pio_sm_set_consecutive_pindirs(pio, sm, clk_pin, 1, true);

    for (uint i = 0; i < CAPTURE_PIN_COUNT; ++i) {
        gpio_set_function(data_pin_base + i, GPIO_FUNC_PIO0);
        gpio_set_dir(data_pin_base + i, GPIO_IN);
    }

    pio_sm_init(pio, sm, offset, &c);
}