#pragma once
#include "pico/stdlib.h"
#include "hardware/pio.h"

void neopixel_init(void);    // does pio_add_program + ws2812_program_init + timer setup
void neopixel_deinit(void);  // stop timer, blank LED, disable SM, remove PIO program
void neopixel_set_rgb(uint8_t r, uint8_t g, uint8_t b);
void neopixel_off(void);
void neopixel_blink_once(uint8_t r, uint8_t g, uint8_t b, uint32_t ms);
void neopixel_blink_multiple_start(uint8_t r, uint8_t g, uint8_t b,
                                    uint32_t on_ms, uint32_t off_ms, uint32_t count);
void neopixel_cancel_timer(void);
