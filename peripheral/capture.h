/* capture.h */
// nice helper functions taken from old version

#pragma once

#include "hardware/pio.h"
#include <stdint.h>

/* ---------- Capture constants ---------- */
#define CAPTURE_PIN_BASE    0u
#define CAPTURE_PIN_COUNT   8u
#define CAPTURE_N_SAMPLES   131072u
#define ADC_CLK_PIN         9u
#define DEFAULT_SAMPLE_HZ   (100u * 1000000u) //100 MHz

/* ---------- Bit-packing helpers ---------- */
uint     bits_packed_per_word(uint pin_count);
uint32_t words_for_samples(uint32_t samples, uint32_t pin_count);

/* ---------- PIO program / state machine ---------- */
uint clk_sample_program_load(PIO pio);
void clk_sample_sm_init(PIO pio, uint sm, uint offset,
                         uint clk_pin, uint data_pin_base, float clkdiv);