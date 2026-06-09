/**
 * Central DAQ — SPI master
 *
 * SPI0 pins (hardware CS):
 *   RX=GPIO0  CS=GPIO1  TX=GPIO3  SCK=GPIO2
 *
 * Peripheral handshake GPIOs:
 *   Periph 0: READY=GPIO26 (out, to periph)  ACK=GPIO27 (in, from periph)
 *   Periph 1: READY=GPIO28 (out, to periph)  ACK=GPIO29 (in, from periph)
 *
 * SD card on SPI1: MISO=GPIO12, MOSI=GPIO11, SCK=GPIO10, CS=GPIO13
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "flash_log.h"

#include "neopixel.h"
#include "sd_config.h"
#include "rtc.h"

// ---------- SPI0 (DAQ comms) ----------
#define DAQ_SPI_INST    spi0
#define SPI_RX_PIN      0
#define SPI_CS_PIN      1
#define SPI_TX_PIN      3
#define SPI_SCK_PIN     6
#define SPI_BAUD_HZ     (6250u * 1000u)   // 6.25kHz

#define A_PIN      27   // Select A (MSB)
#define B_PIN      28   // Select B
#define C_PIN      29   // Select C (LSB)
#define G1_PIN     26   // GPIO26. G2 pin is CS line

#define ARM_IN_PIN      5   // arm input
#define TRIGGER_IN_PIN  4   // trigger input

#define STEP_MS      5000   // dwell time per select combination
#define FLASH_MS     40     // brief flash at step transition

// ---------- External RTC (MCP7940N) ----------
// RTC_ADDR / register map live in rtc.h; these are the board wiring/config.
#define RTC_I2C       i2c0
#define RTC_SDA_PIN   8
#define RTC_SCL_PIN   9
#define RTC_BAUD_HZ   (200 * 1000u) // 400 kHz
static trigger_time_t current_time = {0};

// ---------- Peripherals ----------
// N_PERIPHS now comes from sd_config.h
#define DEFAULT_SAMPLE_HZ   (100u * 1000000u) //100 MHz
uint8_t cap_cnt = 0;

// ---------- Capture buffer (131072 bytes = 128 kB) ----------
#define CAPTURE_BYTES   131072u
static uint8_t rx_buf[CAPTURE_BYTES];

#define BUFFER_SIZE  512    // # 8 bit words
static uint8_t buffer_A[BUFFER_SIZE];
static uint dma_chan;
static dma_channel_config dma_c;
static trigger_time_t trigger_time = {0};

// ---------- run diorecotry tracking ----------
// #define RUN_DIR_NAME_MAX_LEN  16
// static uint32_t g_run_index = 0;
// static char     g_run_dir_name[RUN_DIR_NAME_MAX_LEN] = {0};  // e.g. "RUN_00012"
// static uint32_t g_target_sample_hz = DEFAULT_SAMPLE_HZ;
// static char     g_file_prefix[FILE_PREFIX_MAX_LEN] = DEFAULT_FILE_PREFIX;



// ---------- SPI helpers ----------
static void spi_master_init(void) {
    spi_init(DAQ_SPI_INST, SPI_BAUD_HZ);
    gpio_set_function(SPI_RX_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(SPI_CS_PIN,  GPIO_FUNC_SPI);   // hardware CS
    gpio_set_function(SPI_TX_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
}

// ---------- Periph ACK init ----------
static void arm_and_trigger_init(void) {
    gpio_init(ARM_IN_PIN);
    gpio_set_dir(ARM_IN_PIN, GPIO_IN);
    gpio_pull_down(ARM_IN_PIN);
    gpio_init(TRIGGER_IN_PIN);
    gpio_set_dir(TRIGGER_IN_PIN, GPIO_IN);
    gpio_pull_down(TRIGGER_IN_PIN);

    
}
// ---------- Demux init ----------
static void demux_init(void) {
    gpio_init(A_PIN);
    gpio_set_dir(A_PIN, GPIO_OUT);
    gpio_put(A_PIN, 0);
    gpio_init(B_PIN);
    gpio_set_dir(B_PIN, GPIO_OUT);
    gpio_put(B_PIN, 0);
    gpio_init(C_PIN);
    gpio_set_dir(C_PIN, GPIO_OUT);
    gpio_put(C_PIN, 0);
    gpio_init(G1_PIN);
    gpio_set_dir(G1_PIN, GPIO_OUT);
    gpio_put(G1_PIN, 1);    // G1 IS ALWAYS HIGH
}

// ---------- INITIALIZE CLOCK -----------------
static void rtc_init(void) {
    i2c_init(RTC_I2C, RTC_BAUD_HZ);
    gpio_set_function(RTC_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(RTC_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(RTC_SDA_PIN);
    gpio_pull_up(RTC_SCL_PIN);
    

    if (!mcp_is_running(RTC_I2C)) {
        // Seed an initial time from the firmware build time ("hh:mm:ss") and
        // start the oscillator so the chip keeps track of time.
        const char *bt = __TIME__;
        uint8_t hh = (uint8_t)((bt[0] - '0') * 10 + (bt[1] - '0'));
        uint8_t mm = (uint8_t)((bt[3] - '0') * 10 + (bt[4] - '0'));
        uint8_t ss = (uint8_t)((bt[6] - '0') * 10 + (bt[7] - '0'));
        printf("Setting RTC time to %02u:%02u:%02u\n", hh, mm, ss);
        printf("RTC is not running. Setting time...\n");
        mcp_set_time(RTC_I2C, hh, mm, ss);
    } else {
        printf("RTC is running. Getting time...\n");
        current_time = rtc_get_time(RTC_I2C);
        printf("RTC: %02u:%02u:%02u\n", current_time.hour, current_time.min, current_time.sec);
    }
    mcp_enable_battery(RTC_I2C);
    printf("RTC battery enabled\n");
}


// Probe every 7-bit address and report which ones ACK. Useful to tell apart
// "nothing on the bus" (wiring/power/pull-ups) vs "device at another address".
// static void rtc_bus_scan(void) {
//     printf("I2C scan on RTC bus (SDA=GPIO%u SCL=GPIO%u):\n",
//            RTC_SDA_PIN, RTC_SCL_PIN);
//     int found = 0;
//     for (uint8_t addr = 0x08; addr < 0x78; addr++) {
//         uint8_t dummy = 0;
//         int r = i2c_read_blocking(RTC_I2C, addr, &dummy, 1, false);
//         if (r >= 0) {
//             printf("  device ACKed at 0x%02X\n", addr);
//             found++;
//         }
//     }
//     printf("I2C scan done: %d device(s) found.\n", found);
// }

// static void test_gpio_init(void) {
//     gpio_init(SPI_RX_PIN);
//     gpio_set_dir(SPI_RX_PIN, GPIO_OUT);
//     gpio_put(SPI_RX_PIN, 0);
//     gpio_init(SPI_CS_PIN);
//     gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
//     gpio_put(SPI_CS_PIN, 0);
//     gpio_init(SPI_TX_PIN);
//     gpio_set_dir(SPI_TX_PIN, GPIO_OUT);
//     gpio_put(SPI_TX_PIN, 0);
//     gpio_init(SPI_SCK_PIN);
//     gpio_set_dir(SPI_SCK_PIN, GPIO_OUT);
//     gpio_put(SPI_SCK_PIN, 0);
// }

// static void dma_init(void) {
//     uint dma_chan = dma_claim_unused_channel(true);
//     dma_channel_config dma_c = dma_channel_get_default_config(dma_chan);
//     channel_config_set_read_increment(&dma_c, false);
//     channel_config_set_write_increment(&dma_c, true);
//     channel_config_set_transfer_data_size(&dma_c, DMA_SIZE_8);
//     channel_config_set_dreq(&dma_c, spi_get_dreq(spi0, false)); // paced by SPI RX
// }

// ---------- Main ----------
int main(void) {
    stdio_init_all();
    for (int i = 5; i > 0; i--) {
        sleep_ms(1000);
        if (stdio_usb_connected())
            printf("USB connected, starting in %d...\n", i);
    }
    printf("\n=== Central DAQ ===\n");
    printf("sys_clk = %u Hz\n", clock_get_hz(clk_sys));

    rtc_init(); //for now since we dont have the battery running the rtc in the background

    // if (logic_has_saved_capture()) {
    //     printf("Capture found in flash. Erasing...\n");
    //     logic_erase_saved_capture();
    // }

    // ---------- PIN TOGGLE TEST ----------
    // {
    //     test_gpio_init();
    //     printf("\n--- PIN TOGGLE TEST ---\n");
    //     gpio_put(SPI_RX_PIN, 1);
    //     sleep_ms(4000);
    //     printf("RX pin high for 4 seconds\n");
    //     gpio_put(SPI_RX_PIN, 0);
    //     sleep_ms(500);
    //     printf("RX pin low for 500ms\n");
    //     gpio_put(SPI_CS_PIN, 1);
    //     sleep_ms(4000);
    //     printf("CS pin high for 4 seconds\n");
    //     gpio_put(SPI_CS_PIN, 0);
    //     sleep_ms(500);
    //     printf("CS pin low for 500ms\n");
    //     gpio_put(SPI_TX_PIN, 1);
    //     sleep_ms(4000);
    //     printf("TX pin high for 4 seconds\n");
    //     gpio_put(SPI_TX_PIN, 0);
    //     sleep_ms(500);
    //     printf("TX pin low for 500ms\n");
    //     gpio_put(SPI_SCK_PIN, 1);
    //     sleep_ms(4000);
    //     printf("SCK pin high for 4 seconds\n");
    //     gpio_put(SPI_SCK_PIN, 0);
    //     sleep_ms(500);
    //     printf("SCK pin low for 500ms\n");
    //     while (true) tight_loop_contents();
    // }

    // ---------- ACK PIN TOGGLE TEST ----------
    // {
    //     printf("\n--- ACK PIN TOGGLE TEST ---\n");


    //     gpio_put(periphs[0].ack_gpio, 1);
    //     printf("Asserting ACK pin (GPIO %u) for 5 seconds\n", periphs[0].ack_gpio);
    //     sleep_ms(5000);
    //     gpio_put(periphs[0].ack_gpio, 0);
    //     sleep_ms(500);
    //     gpio_put(periphs[1].ack_gpio, 1);
    //     sleep_ms(4000);
    //     gpio_put(periphs[1].ack_gpio, 0);
    //     sleep_ms(500);
    //     while (true) tight_loop_contents();
    // }
    
    // ---------- DEMUX  TEST ----------
    // {
    //     printf("\n--- DEMUX PIN TOGGLE TEST ---\n");

    //     gpio_init(A_PIN);
    //     gpio_set_dir(A_PIN, GPIO_OUT);
    //     gpio_put(A_PIN, 0);
    //     gpio_init(B_PIN);
    //     gpio_set_dir(B_PIN, GPIO_OUT);
    //     gpio_put(B_PIN, 0);
    //     gpio_init(C_PIN);
    //     gpio_set_dir(C_PIN, GPIO_OUT);
    //     gpio_put(C_PIN, 0);
        
    //     // SPI_CS_PIN (signal / active-high enable) held high the whole time
    //     gpio_init(SPI_CS_PIN);
    //     gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
    //     gpio_put(SPI_CS_PIN, 1);

    //     neopixel_set_rgb(0, 64, 0);  // green: iteration running
    //     printf("Go! Sweeping Y0 -> Y7, %d ms per step.\n", STEP_MS);
    //     stdio_flush();

    //     for (int v = 0; v < 8; v++) {
    //         // 74HC138
    //         int c = (v >> 2) & 1;  
    //         int b = (v >> 1) & 1;
    //         int a =  v       & 1;   

    //         gpio_put(A_PIN, a);
    //         gpio_put(B_PIN, b);
    //         gpio_put(C_PIN, c);

    //         printf("Step %d/8: CBA=%d%d%d  (GP27_A=%d GP28_B=%d GP29_C=%d)  -> Y%d\n",
    //             v + 1, c, b, a, a, b, c, v);
    //         stdio_flush();

    //         // brief white flash to mark the transition, then back to green for the dwell
    //         neopixel_set_rgb(64, 64, 64);
    //         sleep_ms(FLASH_MS);
    //         neopixel_set_rgb(0, 64, 0);
    //         sleep_ms(STEP_MS - FLASH_MS);
    //     }

    //     // Done: drop selects low and turn LED off
    //     gpio_put(A_PIN, 0);
    //     gpio_put(B_PIN, 0);
    //     gpio_put(C_PIN, 0);
    //     gpio_put(SPI_CS_PIN, 0);
    //     neopixel_set_rgb(0, 0, 0);

    //     printf("Sweep complete. Reset the Pico to run again.\n");
    //     stdio_flush();
    //     while (true) tight_loop_contents();

         
    // }

    // ----------  DEMUX PERIPH CONNXN ------------------
    // {
    //     demux_init();
    //     // SPI_CS_PIN (signal / active-high enable) held high the whole time
    //     gpio_init(SPI_CS_PIN);
    //     gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
    //     gpio_put(SPI_CS_PIN, 1);

    //     neopixel_set_rgb(0, 64, 0);  // green: iteration running
    //     printf("Go! Sweeping Y0 -> Y7, %d ms per step.\n", STEP_MS);
    //     stdio_flush();

    //     for (int v = 0; v < 8; v++) {
    //         // 74HC138
    //         int c = (v >> 2) & 1;  
    //         int b = (v >> 1) & 1;
    //         int a =  v       & 1;   

    //         gpio_put(A_PIN, a);
    //         gpio_put(B_PIN, b);
    //         gpio_put(C_PIN, c);

    //         printf("Step %d/8: CBA=%d%d%d  (GP27_A=%d GP28_B=%d GP29_C=%d)  -> Y%d\n",
    //             v + 1, c, b, a, a, b, c, v);
    //         stdio_flush();

    //         // brief white flash to mark the transition, then back to green for the dwell
    //         neopixel_set_rgb(64, 64, 64);
    //         sleep_ms(FLASH_MS);
    //         neopixel_set_rgb(0, 64, 0);
    //         sleep_ms(10000);
    //     }

    //     // Done: drop selects low and turn LED off
    //     gpio_put(A_PIN, 0);
    //     gpio_put(B_PIN, 0);
    //     gpio_put(C_PIN, 0);
    //     gpio_put(SPI_CS_PIN, 0);
    //     neopixel_set_rgb(0, 0, 0);

    //     printf("Sweep complete. Reset the Pico to run again.\n");
    //     stdio_flush();
    //     while (true) tight_loop_contents();
    // }

    // ---------- PIN TOGGLE W DEMUX for just (A, B, C) = (0, 0, 0)----------
    // {
    //     printf("\n--- PIN TOGGLE W DEMUX for just (A, B, C) = (0, 0, 0) ---\n");
    //     demux_init();
    //     test_gpio_init();

    //     gpio_put(A_PIN, 0);
    //     gpio_put(B_PIN, 0);
    //     gpio_put(C_PIN, 0);
    //     gpio_put(G1_PIN, 1);

    //     uint32_t elapsed_ms = 0;
    //     printf("\n--- PIN TOGGLE TEST ---\n");
    //     gpio_put(SPI_RX_PIN, 1);
    //     sleep_ms(4000);
    //     printf("RX pin high for 4 seconds\n");
    //     gpio_put(SPI_RX_PIN, 0);
    //     sleep_ms(500);
    //     printf("RX pin low for 500ms\n");
    //     gpio_put(SPI_CS_PIN, 1);
    //     sleep_ms(4000);
    //     printf("CS pin high for 4 seconds\n");
    //     gpio_put(SPI_CS_PIN, 0);
    //     sleep_ms(500);
    //     printf("CS pin low for 500ms\n");
    //     gpio_put(SPI_TX_PIN, 1);
    //     sleep_ms(4000);
    //     printf("TX pin high for 4 seconds\n");
    //     gpio_put(SPI_TX_PIN, 0);
    //     sleep_ms(500);
    //     printf("TX pin low for 500ms\n");
    //     gpio_put(SPI_SCK_PIN, 1);
    //     sleep_ms(4000);
    //     printf("SCK pin high for 4 seconds\n");
    //     gpio_put(SPI_SCK_PIN, 0);
    //     sleep_ms(500);
    //     printf("SCK pin low for 500ms\n");
    //     printf("\n");
    //     while (true) tight_loop_contents();
        
    // }


    // ---------- ACK SPI DIAGNOSTIC TEST ----------
    // {
    //     printf("\n--- SPI diagnostic ---\n");
    //     neopixel_set_rgb(0, 0, 100); // blue = SPI diagnostic started
    //     sleep_ms(8000); //setup time
    //     uint8_t tx[256] = {0};
    //     uint8_t rx[256] = {0};
    //     printf("Receiving 256 bytes from peripheral...\n");
    //     stdio_flush();
        
    //     for (size_t i = 0; i < N_PERIPHS; i++) {
    //         printf("Asserting READY pin for peripheral %zu: pin %i \n", i, periphs[i].ack_gpio);
    //         gpio_put(periphs[i].ack_gpio, 1);
    //         sleep_us(800);
            
    //         spi_write_read_blocking(DAQ_SPI_INST, tx, rx, 256);
            

    //         printf("Deasserting READY pin for peripheral %zu: pin %i \n", i, periphs[i].ack_gpio);
    //         gpio_put(periphs[i].ack_gpio, 0);
            
    //         for (int j = 0; j < 16; j++) {
    //             int off = 16 * j;
    //             printf("RX[%3d:%3d]: %02X %02X %02X %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X %02X %02X\n",
    //                    off, off + 16,
    //                    rx[off],    rx[off+1],  rx[off+2],  rx[off+3],
    //                    rx[off+4],  rx[off+5],  rx[off+6],  rx[off+7],
    //                    rx[off+8],  rx[off+9],  rx[off+10], rx[off+11],
    //                    rx[off+12], rx[off+13], rx[off+14], rx[off+15]);
    //         }

    //         neopixel_cancel_timer();

    //         bool diag_ok = true;
    //         for (int k = 0; k < 256; k++) {
    //             if (rx[k] != (uint8_t)k) {
    //                 diag_ok = false;
    //                 break;
    //             }
    //         }
    //         if (diag_ok) {
    //             neopixel_blink_once(0, 100, 0, 1000); //green
    //             printf("Diagnostic PASSED\n");
    //         } else {
    //             neopixel_blink_once(100, 0, 0, 1000); //red
    //             printf("Diagnostic FAILED\n");
    //         }

    //         sleep_us(100);
       
    //     }
    //     stdio_flush();

    //     // Halt here until capture loop is enabled
    //     while (true) tight_loop_contents();
    // }

    // ---------- DMA + DENUX DIAGNOSTIC TEST ----------
    // {
    //     printf("\n--- DMA diagnostic ---\n");
    //     dma_init();
    //     spi_master_init();
    //     demux_init();
    //     neopixel_set_rgb(0, 0, 100); // blue = SPI diagnostic started
    //     sleep_ms(4000); //setup time
    //     for (int v = 0; v < N_PERIPHS; v++) {
            
    //         // 74HC138
    //         int c = (v >> 2) & 1;  
    //         int b = (v >> 1) & 1;
    //         int a =  v       & 1;   

    //         gpio_put(A_PIN, a);
    //         gpio_put(B_PIN, b);
    //         gpio_put(C_PIN, c);

    //         //sleep_us(1000);
    //         printf("Chip Select peripheral %d\n", v);
    //         printf("Step %d/8: CBA=%d%d%d  (GP27_A=%d GP28_B=%d GP29_C=%d)  -> Y%d\n",
    //             v + 1, c, b, a, a, b, c, v);
    //         stdio_flush();

    //         memset(buffer_A, 0xDE, BUFFER_SIZE);
    //         dma_channel_configure(dma_chan, &dma_c,
    //             buffer_A,           // write to
    //             &spi_get_hw(spi0)->dr,  // read from
    //             BUFFER_SIZE,         // # 32 bit words

    //             true                // start immediately
    //         );
    //         dma_channel_wait_for_finish_blocking(dma_chan);
        
    //         for (int j = 0; j < 32; j++) {
    //             int off = 16 * j;
    //             printf("RX[%3d:%3d]: %02X %02X %02X %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X %02X %02X\n",
    //                     off, off + 16,
    //                     buffer_A[off],    buffer_A[off+1],  buffer_A[off+2],  buffer_A[off+3],
    //                     buffer_A[off+4],  buffer_A[off+5],  buffer_A[off+6],  buffer_A[off+7],
    //                     buffer_A[off+8],  buffer_A[off+9],  buffer_A[off+10], buffer_A[off+11],
    //                     buffer_A[off+12], buffer_A[off+13], buffer_A[off+14], buffer_A[off+15]);
    //         }
            
    //         bool diag_ok = true;
    //         for (int k = 0; k < 512; k++) {
    //             if (buffer_A[k] != (uint8_t)k) {
    //                 diag_ok = false;
    //                 break;
    //             }
    //         }
    //         sleep_us(100); 
       
    //     }
    //     while (true) tight_loop_contents();
    // }

    // ---------- SD PIN TOGGLE DIAGNOSTIC ----------
    // {
    //     printf("\n--- SD PIN TOGGLE DIAGNOSTIC ---\n");
    //     sd_init();
        // gpio_init(13); gpio_set_dir(13, GPIO_OUT);
        // printf("CS pin high for 4 seconds\n");
        // gpio_put(13, 1);
        // sleep_ms(4000);
        // gpio_put(13, 0);
        // sleep_ms(1000);
        // gpio_init(10); gpio_set_dir(10, GPIO_OUT);
        // printf("SCK pin high for 4 seconds\n");
        // gpio_put(10, 1);
        // sleep_ms(4000);
        // gpio_put(10, 0);
        // sleep_ms(1000);
        // gpio_init(11); gpio_set_dir(11, GPIO_OUT);
        // printf("MOSI pin high for 4 seconds\n");
        // gpio_put(11, 1);
        // sleep_ms(4000);
        // gpio_put(11, 0);
        // sleep_ms(1000);
        // gpio_init(8); gpio_set_dir(8, GPIO_OUT);
        // printf("MISO pin high for 15 seconds\n");
        // gpio_put(8, 1);
        // sleep_ms(15000);
        // gpio_put(8, 0);
        // sleep_ms(1000);
        // printf("All pins low\n");
        // gpio_put(13, 0);
        // gpio_put(10, 0);
        // gpio_put(11, 0);
        // gpio_put(8, 0);
        // while (true) tight_loop_contents();
    // }
    // // ---------- SD DIAGNOSTIC ----------
    // {
    //     printf("\n--- SD diagnostic ---\n");
    //     sd_init();
    //     printf("SD initialized\n");

    //     uint8_t rx_buf[256] = {0};
    //     for (int i = 0; i < 256; i++) rx_buf[i] = (uint8_t)i;
    //     printf("Writing 256 bytes to SD...\n");
    //     bool transfer = write_capture_to_sd(rx_buf, 256);
    //     if (!transfer) {
    //         printf("ERROR: SD write failed\n");
    //         neopixel_set_rgb(100, 0, 0); //red
    //         while (true) tight_loop_contents();
    //     }
    //     neopixel_set_rgb(0, 100, 0); //green
    //     printf("SD write successful\n");
    //     sd_unmount();
    //     while (true) tight_loop_contents();
    // }
    //---------- SPI DIAGNOSTIC ----------
    // {
    //     printf("\n--- SPI diagnostic ---\n");
    //     gpio_init(SPI_CS_PIN);
    //     gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
    //     gpio_put(SPI_CS_PIN, 1);
    //     demux_init();
    //     sd_init();
    //     printf("SD initialized\n");
    //     neopixel_set_rgb(0, 0, 100); // blue = SPI diagnostic started
    //     sleep_ms(2000); //setup time
    //     // uint8_t tx[256] = {0};
    //     uint8_t rx[256] = {0};
    //     // uint8_t junk[92] = {0};
    //     printf("Receiving 256 bytes from peripheral...\n");
    //     stdio_flush();
        
    //     for (int v = 0; v < N_PERIPHS; v++) {
            
    //         // 74HC138
    //         int c = (v >> 2) & 1;  
    //         int b = (v >> 1) & 1;
    //         int a =  v       & 1;   
    //         gpio_put(A_PIN, a);
    //         gpio_put(B_PIN, b);
    //         gpio_put(C_PIN, c);

    //         // ACK pulse: CS low for 100 µs through demux to selected peripheral  
    //         gpio_put(SPI_CS_PIN, 0);
    //         sleep_us(100);
    //         gpio_put(SPI_CS_PIN, 1);
    //         printf("ACK pulsed to peripheral %d\n", v);

    //         // Init SPI master (takes over CS pin as hardware function)
    //         spi_master_init();

    //         // Give peripheral time to init SPI and pre-load TX FIFO
    //         sleep_ms(1);

    //         printf("Collecting from peripheral %d\n", v);
    //         spi_read_blocking(DAQ_SPI_INST, 0, rx, 256);
    //         printf("Periph %d: transfer done\n", v);
    //         spi_deinit(DAQ_SPI_INST);
    //         gpio_init(SPI_CS_PIN);
    //         gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
    //         gpio_put(SPI_CS_PIN, 1);
            
    //         for (int j = 0; j < 16; j++) {
    //             int off = 16 * j;
    //             printf("RX[%3d:%3d]: %02X %02X %02X %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X %02X %02X\n",
    //                     off, off + 16,
    //                     rx[off],    rx[off+1],  rx[off+2],  rx[off+3],
    //                     rx[off+4],  rx[off+5],  rx[off+6],  rx[off+7],
    //                     rx[off+8],  rx[off+9],  rx[off+10], rx[off+11],
    //                     rx[off+12], rx[off+13], rx[off+14], rx[off+15]);
    //         }
            
    //         bool diag_ok = true;
    //         for (int k = 0; k < 256; k++) {
    //             if (rx[k] != (uint8_t)k) {
    //                 diag_ok = false;
    //                 break;
    //             }
    //         }

    //         neopixel_cancel_timer();

    //         if (diag_ok) {
    //             neopixel_blink_once(0, 100, 0, 1000); //green
    //             printf("Diagnostic PASSED\n");
    //         } else {
    //             neopixel_blink_once(100, 0, 0, 1000); //red
    //             printf("Diagnostic FAILED\n");
    //         }
        
    //         stdio_flush();
    //         // brief white flash to mark the transition, then back to green for the dwell
    //         neopixel_set_rgb(64, 64, 64);
    //         sleep_ms(FLASH_MS);
    //         neopixel_set_rgb(0, 64, 0);
            
    //         // ----------- WRITE TO SD CARD -------------
    //         printf("Writing to SD card\n");
    //         bool transfer = write_capture_to_sd(rx, sizeof(rx));
    //         if (!transfer) {
    //             printf("ERROR: SD write failed\n");
    //             neopixel_set_rgb(100, 0, 0); //red
    //         }
    //         neopixel_set_rgb(0, 100, 0); //green
    //         printf("SD write successful\n");

    //         sleep_ms(STEP_MS - FLASH_MS);
            
    //     }
    //     sd_unmount();
    //     while (true) tight_loop_contents();
        
    // }
    // ---------- spi DIAGNOSTIC temp ----------
    // {
    //     printf("\n--- SPI diagnostic ---\n");
    //     gpio_init(SPI_CS_PIN);
    //     gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
    //     gpio_put(SPI_CS_PIN, 1);
    //     demux_init();
    //     sd_init();
    //     printf("SD initialized\n");
    //     neopixel_set_rgb(0, 0, 100); // blue = SPI diagnostic started

    //     while(!gpio_get(ARM_IN_PIN)) tight_loop_contents();
    //     neopixel_blink_once(50, 50, 50, 500); // grey = arm pulse initiated
    //     printf("ARM pulse received\n");

    //     // 2. wait for trigger pulse (100 ms pulse high)
    //     while(!gpio_get(TRIGGER_IN_PIN)) tight_loop_contents();
    //     neopixel_blink_once(50, 50, 50, 500); // grey = trigger pulse initiated
    //     printf("TRIGGER pulse received\n");
    //     // uint8_t tx[256] = {0};

    //     sleep_ms(2000); //setup time

    //     uint8_t rx[256] = {0};
    //     // uint8_t junk[92] = {0};
    //     printf("Receiving 256 bytes from peripheral...\n");
    //     stdio_flush();
        
    //     for (int v = 0; v < N_PERIPHS; v++) {
            
    //         // 74HC138
    //         int c = (v >> 2) & 1;  
    //         int b = (v >> 1) & 1;
    //         int a =  v       & 1;   
    //         gpio_put(A_PIN, a);
    //         gpio_put(B_PIN, b);
    //         gpio_put(C_PIN, c);

    //         // ACK pulse: CS low for 100 µs through demux to selected peripheral  
    //         gpio_put(SPI_CS_PIN, 0);
    //         sleep_us(100);
    //         gpio_put(SPI_CS_PIN, 1);
    //         printf("ACK pulsed to peripheral %d\n", v);

    //         // Init SPI master (takes over CS pin as hardware function)
    //         spi_master_init();

    //         // Give peripheral time to init SPI and pre-load TX FIFO
    //         sleep_ms(1);

    //         printf("Collecting from peripheral %d\n", v);
    //         spi_read_blocking(DAQ_SPI_INST, 0, rx, 256);
    //         printf("Periph %d: transfer done\n", v);
    //         spi_deinit(DAQ_SPI_INST);
    //         gpio_init(SPI_CS_PIN);
    //         gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
    //         gpio_put(SPI_CS_PIN, 1);
            
    //         for (int j = 0; j < 16; j++) {
    //             int off = 16 * j;
    //             printf("RX[%3d:%3d]: %02X %02X %02X %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X %02X %02X\n",
    //                     off, off + 16,
    //                     rx[off],    rx[off+1],  rx[off+2],  rx[off+3],
    //                     rx[off+4],  rx[off+5],  rx[off+6],  rx[off+7],
    //                     rx[off+8],  rx[off+9],  rx[off+10], rx[off+11],
    //                     rx[off+12], rx[off+13], rx[off+14], rx[off+15]);
    //         }
            
    //         bool diag_ok = true;
    //         for (int k = 0; k < 256; k++) {
    //             if (rx[k] != (uint8_t)k) {
    //                 diag_ok = false;
    //                 break;
    //             }
    //         }

    //         neopixel_cancel_timer();

    //         if (diag_ok) {
    //             neopixel_blink_once(0, 100, 0, 1000); //green
    //             printf("Diagnostic PASSED\n");
    //         } else {
    //             neopixel_blink_once(100, 0, 0, 1000); //red
    //             printf("Diagnostic FAILED\n");
    //         }
        
    //         stdio_flush();
    //         // brief white flash to mark the transition, then back to green for the dwell
    //         neopixel_set_rgb(64, 64, 64);
    //         sleep_ms(FLASH_MS);
    //         neopixel_set_rgb(0, 64, 0);
            
    //         // ----------- WRITE TO SD CARD -------------
    //         printf("Writing to SD card\n");
    //         bool transfer = write_capture_to_sd(rx, sizeof(rx));
    //         if (!transfer) {
    //             printf("ERROR: SD write failed\n");
    //             neopixel_set_rgb(100, 0, 0); //red
    //         }
    //         neopixel_set_rgb(0, 100, 0); //green
    //         printf("SD write successful\n");

    //         // sleep_ms(STEP_MS - FLASH_MS);
            
    //     }
    //     sd_unmount();
    //     while (true) tight_loop_contents();
        
    // }

    // ---------- TEST RTC ------------
    // {
    //     printf("\n--- TEST RTC ---\n");
    //     rtc_init();
    //     rtc_bus_scan();
    //     sleep_ms(1000);
    //     rtc_print_time();
    //     sleep_ms(1000);
    //     rtc_print_time();
    //     sleep_ms(4000);
    //     rtc_print_time();
    //     while (true) tight_loop_contents();
    // }
        
    // ---------- CAPTURE LOOP ----------

    // Mount SD card for logging
    if (!sd_init()) {
        neopixel_set_rgb(100, 0, 0);
        printf("ERROR: SD mount failed. Halting.\n");
        while (true) tight_loop_contents();
    }
    // neopixel_set_rgb(0, 0, 50);   // dim blue = SD ready, waiting

    printf("Number of peripherals: %u\n", sd_get_num_periphs());
    demux_init();

    while (true) {
        init_run_directory();
        arm_and_trigger_init();
        neopixel_init();  // dim orange = starting


        gpio_init(SPI_TX_PIN);
        gpio_set_dir(SPI_TX_PIN, GPIO_OUT);
        gpio_put(SPI_TX_PIN, 0);
        // CS held high as plain GPIO until first ACK pulse (never float)

        gpio_init(SPI_CS_PIN);
        gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
        gpio_put(SPI_CS_PIN, 1);

        // 1. wait for universal ARM signal (100 ms pulse high)
        while(!gpio_get(ARM_IN_PIN)) tight_loop_contents();
        gpio_put(SPI_TX_PIN, 1);
        sleep_ms(1000);
        gpio_put(SPI_TX_PIN, 0);
        printf("ARM pulse received\n");
        neopixel_blink_once(50, 50, 50, 500); // grey = arm pulse initiated
        
        sleep_ms(1000);
        gpio_put(SPI_TX_PIN, 0);
        sd_init();
        

        // 2. wait for trigger pulse (100 ms pulse high)
        while(!gpio_get(TRIGGER_IN_PIN)) tight_loop_contents();
        trigger_time = rtc_get_time(RTC_I2C);
        printf("RTC: %02u:%02u:%02u\n",
           trigger_time.hour, trigger_time.min, trigger_time.sec);
        // neopixel_blink_once(50, 50, 50, 500); // grey = trigger pulse initiated
        printf("TRIGGER pulse received\n");

        // 3. Wait for capture + bit-reversal to complete on peripherals
        sleep_ms(3000);

        // 4. Collect from each peripheral via demux + ACK/SPI
        
        //temp
        uint8_t rx[256] = {0};
        stdio_flush();

        for (int v = 0; v < sd_get_num_periphs(); v++) {
            // Select peripheral via demux (CS is high/idle)
            int c_sel = (v >> 2) & 1;
            int b_sel = (v >> 1) & 1;
            int a_sel =  v       & 1;
            gpio_put(A_PIN, a_sel);
            gpio_put(B_PIN, b_sel);
            gpio_put(C_PIN, c_sel);
            sleep_us(10);   // demux settle
            
            // ACK pulse: CS low for 100 µs through demux to selected peripheral
            gpio_put(SPI_CS_PIN, 0);
            sleep_us(100);
            gpio_put(SPI_CS_PIN, 1);
            printf("ACK pulsed to peripheral %d\n", v);

            // Init SPI master (takes over CS pin as hardware function)
            spi_master_init();

            // Give peripheral time to init SPI and pre-load TX FIFO
            sleep_ms(1);

            //temp
            
            printf("Collecting from peripheral %d\n", v);
            spi_read_blocking(DAQ_SPI_INST, 0, rx_buf, CAPTURE_BYTES);
            // spi_read_blocking(DAQ_SPI_INST, 0, rx, 256);
            // printf("Periph %d: transfer done\n", v);
            // for(int i = 0; i < 16; i++) {
            //     int off = 16 * i;
            //     printf("RX[%3d:%3d]: %02X %02X %02X %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X %02X %02X\n",
            //             off, off + 16,
            //             rx[off],    rx[off+1],  rx[off+2],  rx[off+3],
            //             rx[off+4],  rx[off+5],  rx[off+6],  rx[off+7],
            //             rx[off+8],  rx[off+9],  rx[off+10], rx[off+11],
            //             rx[off+12], rx[off+13], rx[off+14], rx[off+15]);
            // }


            // Deinit SPI, return CS to GPIO OUT HIGH for next ACK pulse
            spi_deinit(DAQ_SPI_INST);
            gpio_init(SPI_CS_PIN);
            gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
            gpio_put(SPI_CS_PIN, 1);   // CS idle high — demux deselected

            // Print sampled bytes from rx_buf as full 8-bit binary values.
            // // rx_buf is the raw byte stream from the peripheral — sample s is rx_buf[s].
            // {
            //     for (int row = 0; row < 50; row++) {
            //         uint32_t first_step = (uint32_t)row * 20;
            //         printf("[%6lu-%6lu]:",
            //             (unsigned long)(first_step * 50u),
            //             (unsigned long)((first_step + 19) * 50u));
            //         for (int col = 0; col < 40; col++) {
            //             uint32_t s = (first_step + (uint32_t)col) * 50u;
            //             if (s < CAPTURE_BYTES) {
            //                 uint8_t b = rx_buf[s];
            //                 char bits[9];
            //                 for (int k = 0; k < 8; k++) {
            //                     bits[k] = (b & (0x80u >> k)) ? '1' : '0';
            //                 }
            //                 bits[8] = '\0';
            //                 printf(" %s", bits);
            //             } else {
            //                 printf(" --------");
            //             }
            //         }
            //         printf("\n");
            //     }
            // }

            // Find received bytes where bit 7 != bit 6 (sanity check on rx_buf)
            // {
            //     const uint8_t *bytes = rx_buf;
            //     uint32_t mismatches = 0;
            //     uint32_t scan_limit = 10000u;
            //     if (scan_limit > CAPTURE_BYTES) scan_limit = CAPTURE_BYTES;
            //     printf("Scanning first %lu bytes for bit7 != bit6...\n",
            //            (unsigned long)scan_limit);
            //     for (uint32_t i = 0; i < scan_limit; i++) {
            //         uint8_t b = bytes[i];
            //         if (((b >> 7) & 1u) != ((b >> 6) & 1u)) {
            //             char bits[9];
            //             for (int k = 0; k < 8; k++) {
            //                 bits[k] = (b & (0x80u >> k)) ? '1' : '0';
            //             }
            //             bits[8] = '\0';
            //             printf("  idx=%6lu  byte=0x%02X  bin=%s\n",
            //                    (unsigned long)i, b, bits);
            //             mismatches++;
            //         }
            //     }
            //     printf("Done. %lu mismatches in %lu bytes.\n",
            //            (unsigned long)mismatches, (unsigned long)scan_limit);
            // }

            bool transfer = write_capture_to_sd(rx_buf, CAPTURE_BYTES, DEFAULT_SAMPLE_HZ, &trigger_time, v, cap_cnt);

            neopixel_init();
            if (!transfer) {
                printf("ERROR: SD write failed for periph %d\n", v);
                neopixel_blink_once(100, 0, 0, 1000); //red  
            }
            else {
                neopixel_blink_once(0, 100, 0, 1000); //green
            }
            neopixel_deinit();

        }

        printf("Cycle complete\n");
        sd_unmount();
        sleep_ms(1000);
        // neopixel_set_rgb(0, 100, 0); //green
        sleep_ms(1000);

        cap_cnt++;

    }
}