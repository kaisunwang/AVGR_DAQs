/**
 * PIO logic analyser with ADC clock (sideset) + SPI transfer to central DAQ
 * - Clock output on GPIO9 (ADC_CLK_PIN)
 * - 8-bit sample bus on GPIO 0..7
 */

 #include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/clocks.h"
#include "hardware/spi.h"

#include "spi_slave_config.h"
#include "neopixel.h"
#include "capture.h"

// ---------- Config ----------
const uint TRIGGER_PIN   = 11; 
const uint ARM_PIN       = SPI_RX_PIN;
const bool TRIGGER_LEVEL = true;

static uint32_t g_capture_index    = 0;
static uint32_t g_target_sample_hz = DEFAULT_SAMPLE_HZ;

// Reverse all 8 bits in a byte: b7..b0 -> b0..b7
static inline uint8_t reverse_byte(uint8_t x) {
    x = (x >> 4) | (x << 4);
    x = ((x & 0xCC) >> 2) | ((x & 0x33) << 2);
    x = ((x & 0xAA) >> 1) | ((x & 0x55) << 1);
    return x;
}

// ---------- SPI transfer (CS-triggered protocol) ----------
// Flow:
//   1. Wait for central to assert CS (CS_PIN goes LOW)
//   2. Init SPI1 hardware
//   3. Stream full capture buffer to central
//   4. Deinit SPI1


static void test_gpio_init(void) {
    gpio_init(SPI_RX_PIN);
    gpio_set_dir(SPI_RX_PIN, GPIO_IN);
    gpio_put(SPI_RX_PIN, 0);
    gpio_init(SPI_CS_PIN);
    gpio_set_dir(SPI_CS_PIN, GPIO_IN);
    gpio_put(SPI_CS_PIN, 0);
    gpio_init(SPI_TX_PIN);
    gpio_set_dir(SPI_TX_PIN, GPIO_IN);
    gpio_put(SPI_TX_PIN, 0);
    gpio_init(SPI_SCK_PIN);
    gpio_set_dir(SPI_SCK_PIN, GPIO_IN);
    gpio_put(SPI_SCK_PIN, 0);
}

static void arm_and_trigger_init(void) {
    gpio_init(ARM_PIN);
    gpio_set_dir(ARM_PIN, GPIO_IN);
    gpio_pull_down(ARM_PIN);
    gpio_init(TRIGGER_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_IN);
    gpio_pull_down(TRIGGER_PIN);
}

// ---------- Main ----------
int main(void) {

    stdio_init_all();
    for (int i = 3; i > 0; i--) {
        sleep_ms(1000);
        if (stdio_usb_connected())
            printf("USB connected, starting in %d...\n", i);
    }
    printf("\n=== PIO Logic Analyser (peripheral) ===\n");
    printf("sys_clk = %u Hz\n", clock_get_hz(clk_sys));

    gpio_init(ACK_PIN);
    gpio_set_dir(ACK_PIN, GPIO_IN);
    gpio_pull_down(ACK_PIN);


    // ---------- PIN TOGGLE TEST ----------
//    {
//         printf("\n--- PIN TOGGLE TEST ---\n");
//         test_gpio_init();
//         uint32_t elapsed_ms = 0;
//         while (elapsed_ms < 18000) {
//             printf("\rPERIPHERAL  TX=%d  CS=%d  RX=%d  SCK=%d   ",
//                 gpio_get(SPI_TX_PIN),
//                 gpio_get(SPI_CS_PIN),
//                 gpio_get(SPI_RX_PIN),
//                 gpio_get(SPI_SCK_PIN));
//             fflush(stdout);
//             sleep_ms(100);
//             elapsed_ms += 100;
//         }
//         printf("\n");
//         while (true) tight_loop_contents();
//     }

    //---------- ACK PIN TOGGLE TEST ----------
    // printf("\n--- PIN TOGGLE TEST ---\n");
    // {
    //     uint32_t elapsed_ms = 0;
    //     while (elapsed_ms < 18000) {
    //         printf("\rPERIPHERAL  ACK (GPIO%i): %i  ",
    //             ACK_PIN,
    //             gpio_get(ACK_PIN)
    //         );
    //         fflush(stdout);
    //         sleep_ms(1000);
    //         elapsed_ms += 1000;
    //     }
    //     printf("\n");
    //     while (true) tight_loop_contents();
    // }

    // ---------- ACK SPI DIAGNOSTIC TEST ----------
    // {
    //     printf("\n--- SPI diagnostic ---\n");
    //     printf("Waiting for central to be ready...\n");
    //     stdio_flush();
    //     uint8_t tx[256] = { 0 };
    //     for (int i = 0; i < 256; i++) tx[i] = (uint8_t)i;
    //     uint8_t rx[256] = { 0 };
    //     printf("Sending 256 bytes (0x00..0xFF)...\n");
    //     while(!gpio_get(ACK_PIN)) tight_loop_contents();
    //     spi_slave_init();
    //     spi_write_read_blocking(SPI_SLAVE_INST, tx, rx, 256);
    //     printf("Diagnostic TX done\n");
    //     stdio_flush();
    //     spi_slave_deinit();
    //     while (true) tight_loop_contents();
    // }

    // ----------  DEMUX PERIPH CONNXN ------------------
    // {
    //     printf("\n--- DEMUX PIN TOGGLE TEST ---\n");
    //     test_gpio_init();
    //     uint32_t elapsed_ms = 0;
    //     while (elapsed_ms < 20000) {
    //         printf("\rPERIPHERAL  CS=%d ", gpio_get(SPI_CS_PIN));
    //         fflush(stdout);
    //         sleep_ms(100);
    //         elapsed_ms += 100;
    //     }
    //     printf("\n");
    //     while (true) tight_loop_contents();
    // }

    // ---------- PIN TOGGLE W DEMUX ----------
    // {
    //     printf("\n--- PIN TOGGLE W DEMUX ---\n");
    //     test_gpio_init();
    //     uint32_t elapsed_ms = 0;
    //     while (elapsed_ms < 18000) {
    //         printf("\rPERIPHERAL  TX=%d  CS=%d  RX=%d  SCK=%d   ",
    //             gpio_get(SPI_TX_PIN),
    //             gpio_get(SPI_CS_PIN),
    //             gpio_get(SPI_RX_PIN),
    //             gpio_get(SPI_SCK_PIN));
    //         fflush(stdout);
    //         sleep_ms(100);
    //         elapsed_ms += 100;
    //     }
    //     printf("\n");
    //     while (true) tight_loop_contents();
    // }

    // ---------- SPI DIAGNOSTIC ----------
    // {
    //     gpio_init(TRIGGER_PIN);
    //     gpio_set_dir(TRIGGER_PIN, GPIO_IN);
    //     gpio_pull_down(TRIGGER_PIN);

    //     gpio_init(ARM_PIN);
    //     gpio_set_dir(ARM_PIN, GPIO_IN);
    //     gpio_pull_down(ARM_PIN);

    //     printf("\n--- SPI diagnostic ---\n");
    //     printf("Waiting for central to be ready...\n");
    //     neopixel_set_rgb(0, 0, 100); // blue = waiting for ARM
    //     // 1. wait for universal ARM signal (100 ms pulse high)
    //     while(!gpio_get(ARM_PIN)) tight_loop_contents();
    //     neopixel_blink_once(50, 50, 50, 500); // grey = arm pulse initiated
    //     printf("ARM pulse received\n");

    //     // 2. wait for trigger pulse (100 ms pulse high)
    //     while(!gpio_get(TRIGGER_PIN)) tight_loop_contents();
    //     neopixel_blink_once(50, 50, 50, 500); // grey = trigger pulse initiated
    //     printf("TRIGGER pulse received\n");
        
    //     uint8_t tx[256] = { 0 };
    //     for (int i = 0; i < 256; i++) tx[i] = (uint8_t)i;
    //     printf("Waiting for central to be ready...\n");
    //     while(gpio_get(SPI_CS_PIN) == 1) tight_loop_contents();
    //     spi_slave_init();
    //     spi_write_blocking(SPI_SLAVE_INST, tx, 256);
    //     spi_slave_deinit();

    //     printf("Diagnostic TX done\n");
    //     stdio_flush();
    //     while (true) tight_loop_contents();
    // }

    // ---------- CAPTURE SETUP ----------

    printf("PIO DAQ: Initialising\n");
    set_sys_clock_khz(200000, true);
    neopixel_init();
    printf("Config: SAMPLE_HZ=%u\n", g_target_sample_hz);

    uint32_t buf_size_words = words_for_samples(CAPTURE_N_SAMPLES, CAPTURE_PIN_COUNT);
    uint32_t *capture_buf   = (uint32_t *)malloc(buf_size_words * sizeof(uint32_t));
    hard_assert(capture_buf);

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS |
                            BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    PIO  pio = pio0;
    uint sm  = 0;

    uint32_t f_sys  = clock_get_hz(clk_sys);
    float clkdiv    = (float)f_sys / (2.0f * (float)g_target_sample_hz);

    uint prog_off = clk_sample_program_load(pio);
    clk_sample_sm_init(pio, sm, prog_off, ADC_CLK_PIN, CAPTURE_PIN_BASE, clkdiv);

    uint dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    uint32_t sample_hz = (uint32_t)((float)f_sys / (2.0f * clkdiv) + 0.5f);
    printf("Configured sample rate ~%u Hz (clkdiv=%.3f)\n", sample_hz, clkdiv);

    // set_sys_clock_khz(124, true);
    // ---------- Capture loop ----------
    while (true) {
        //initialize ARM and TRIGGER pins
        arm_and_trigger_init();
        neopixel_init();

        //initialize ACK (CS) pin
        gpio_init(SPI_CS_PIN);
        gpio_set_dir(SPI_CS_PIN, GPIO_IN);
        gpio_pull_down(SPI_CS_PIN);

        clock_configure(
            clk_peri,
            0, // glitchless mux not used for clk_peri
            CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
            125000000, // input Hz
            125000000  // output Hz (after divider)
        ); // else the transfer defaults to some bullshit where each byte is frameshifted down and the most significant bit is the same


        dma_channel_configure(dma_chan, &c,
            capture_buf,
            &pio->rxf[sm],
            buf_size_words,
            false
        );

        pio_sm_set_enabled(pio, sm, false);
        pio_sm_clear_fifos(pio, sm);
        pio_sm_restart(pio, sm);
        pio_sm_exec(pio, sm, pio_encode_wait_gpio(TRIGGER_LEVEL, TRIGGER_PIN));

        printf("Waiting for ARM pin to go high...\n");
        // neopixel_set_rgb(0, 0, 100); // blue = waiting for ARM
        while (!gpio_get(ARM_PIN)) tight_loop_contents();
        neopixel_blink_once(50, 50, 50, 500); // grey = armed
        neopixel_deinit();

        printf("Arming trigger for capture %lu...\n", (unsigned long)g_capture_index);
        g_capture_index++;

        dma_channel_start(dma_chan);
        pio_sm_set_enabled(pio, sm, true);
        dma_channel_wait_for_finish_blocking(dma_chan);
        pio_sm_set_enabled(pio, sm, false);
        printf("Capture complete.\n");
        // neopixel_blink_once(50, 50, 50, 500); // grey = triggered + capture complete

        uint8_t *bytes  = (uint8_t *)capture_buf; // recast as uint8_t rather than uint32_t
        // Bit-reverse every captured byte in place (treats capture_buf as a byte stream).
        {
            uint32_t  total_bytes = buf_size_words * sizeof(uint32_t);
            for (uint32_t i = 0; i < total_bytes; i++) {
                bytes[i] = reverse_byte(bytes[i]);
            }
        }


        printf("Capture complete.\n");
         // print # samples recorded in capture_buf
        printf("Number of samples recorded: %lu\n", (unsigned long)buf_size_words);
        
        printf("Waiting for CS low from central...\n");
        printf("System clock: %u Hz\n", clock_get_hz(clk_sys));
        
        // uint8_t tx[256] = { 0 };
        // for (int i = 0; i < 256; i++) tx[i] = (uint8_t)i;
        // printf("Waiting for central to be ready...\n");
        // while(gpio_get(SPI_CS_PIN) == 1) tight_loop_contents();
        // spi_slave_init();
        // spi_write_blocking(SPI_SLAVE_INST, tx, 256);
        // spi_slave_deinit();
        // printf("Transfer to central complete.\n");
        // for(int i = 0; i < 16; i++) {
        //     int off = 16 * i;
        //     printf("TX[%3d:%3d]: %02X %02X %02X %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X %02X %02X\n",
        //             off, off + 16,
        //             tx[off],    tx[off+1],  tx[off+2],  tx[off+3],
        //             tx[off+4],  tx[off+5],  tx[off+6],  tx[off+7],
        //             tx[off+8],  tx[off+9],  tx[off+10], tx[off+11],
        //             tx[off+12], tx[off+13], tx[off+14], tx[off+15]);
        // }

        while (gpio_get(SPI_CS_PIN)) tight_loop_contents();
        spi_slave_init();
        // if (!spi_central_ready()) {
        //     printf("ERROR: SPI not ready. Halting.\n");
        //     while (true) {
        //         neopixel_set_rgb(100, 0, 0); //red = SPI not ready
        //         //flash procedure here
        //         tight_loop_contents();
        //     }
        // }
        spi_write_blocking(SPI_SLAVE_INST, bytes, 131072u);
        spi_slave_deinit();
        printf("Transfer to central complete.\n");
        // neopixel_blink_once(0, 100, 0, 1000); // green = sent

    }
}


// Bit-reverse each 8-bit sample (match reference)
// {
//     uint32_t rec_bits = bits_packed_per_word(CAPTURE_PIN_COUNT);
//     uint32_t samples_per_word = rec_bits / CAPTURE_PIN_COUNT;
//     for (uint32_t i = 0; i < buf_size_words; i++) {
//         uint32_t w = capture_buf[i];
//         uint32_t new_w = 0;
//         for (uint32_t s = 0; s < samples_per_word; s++) {
//             uint32_t shift = 24 - 8 * s;
//             uint8_t samp = (w >> shift) & 0xFF;
//             uint8_t rev  = reverse_byte(samp);
//             new_w |= ((uint32_t)rev) << shift;
//         }
//         capture_buf[i] = new_w;
//     }
// }