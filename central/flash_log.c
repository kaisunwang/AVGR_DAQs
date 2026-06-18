/**
 * sd capture file logging.
 * not used in june avgr tests
 */

#include "flash_log.h"
#include <stdio.h>
#include "ff.h"
#include "f_util.h"

static FIL     g_file;
static bool    g_open       = false;
static uint32_t g_file_index = 0;



// bool flash_log_open_new(void) {
//     if (g_open) flash_log_close();

//     char fname[32];
//     snprintf(fname, sizeof(fname), "0:cap_%04lu.bin", (unsigned long)g_file_index);

//     FRESULT fr = f_open(&g_file, fname, FA_WRITE | FA_CREATE_ALWAYS);
//     if (fr != FR_OK) {
//         printf("ERROR: f_open(%s) failed (%s)\n", fname, FRESULT_str(fr));
//         return false;
//     }

//     g_open = true;
//     printf("Opened %s for writing\n", fname);
//     return true;
// }

// bool flash_log_write(const uint8_t *data, uint32_t len) {
//     if (!g_open) return false;

//     UINT written;
//     FRESULT fr = f_write(&g_file, data, len, &written);
//     if (fr != FR_OK || written != len) {
//         printf("ERROR: f_write failed (wrote %u/%lu, %s)\n",
//                written, (unsigned long)len, FRESULT_str(fr));
//         return false;
//     }
//     return true;
// }

// void flash_log_close(void) {
//     if (!g_open) return;
//     f_close(&g_file);
//     g_open = false;
//     g_file_index++;
// }

// uint32_t flash_log_get_index(void) {
//     return g_file_index;
// }


// static inline uint8_t reverse_byte(uint8_t x) {
//     x = (x >> 4) | (x << 4);
//     x = ((x & 0xCC) >> 2) | ((x & 0x33) << 2);
//     x = ((x & 0xAA) >> 1) | ((x & 0x55) << 1);
//     return x;
// }

// bool logic_has_saved_capture(void) { // check if there is a capture in flash
//     const logic_log_header_t *hdr = (const logic_log_header_t *)(XIP_BASE + LOG_OFFSET);
//     if (hdr->magic != LOG_MAGIC || hdr->version != 1) return false;
//     if (hdr->word_count == 0 || hdr->word_count > (LOG_REGION_SIZE / 4)) return false;
//     return true;
// }

// void logic_dump_saved_capture_hex(void) { // retrieve the previously stored header and display on printf
//     const logic_log_header_t *hdr = (const logic_log_header_t *)(XIP_BASE + LOG_OFFSET);
//     if (hdr->magic != LOG_MAGIC) { printf("NOLOG\n"); return; }

//     uint32_t expected_words = words_for_samples(hdr->sample_count, hdr->pin_count);
//     uint32_t max_words = (LOG_REGION_SIZE - HEADER_STORE_LEN) / 4;
//     uint32_t words = hdr->word_count;
//     if (words > expected_words) words = expected_words;
//     if (words > max_words) words = max_words;

//     printf("HDR magic=LAC1 ver=%u samples=%u pins=%u base=%u hz=%u trig=%u/%u words=%u\n",
//         hdr->version, hdr->sample_count, hdr->pin_count, hdr->pin_base, hdr->sample_hz,
//         hdr->trigger_pin, hdr->trigger_level, words);

//     const uint32_t *data = (const uint32_t *)(XIP_BASE + LOG_OFFSET + HEADER_STORE_LEN);
//     for (uint32_t i = 0; i < words; ++i) {
//         printf("%08X\n", data[i]);
//     }
//     printf("END\n");
// }

// void logic_save_capture_to_flash_raw(const uint32_t *cap_words, // save the capture to flash
//     uint32_t sample_count,
//     uint32_t word_count,
//     uint32_t pin_base,
//     uint32_t pin_count,
//     uint32_t sample_hz,
//     uint32_t trigger_pin,
//     uint32_t trigger_level)
// {
//     logic_log_header_t hdr = {0};
//     hdr.magic         = LOG_MAGIC;
//     hdr.version       = 1;
//     hdr.hdr_len       = sizeof(hdr);
//     hdr.sample_count  = sample_count;  // logical samples
//     hdr.word_count    = word_count;    // 32-bit words

//     uint8_t header_block[HEADER_STORE_LEN];
//     memset(header_block, 0xFF, sizeof(header_block));
//     memcpy(header_block, &hdr, sizeof(hdr));

//     const uint32_t data_bytes = word_count * sizeof(uint32_t);
//     const uint32_t prog_len   = HEADER_STORE_LEN + align_up(data_bytes, HEADER_STORE_LEN);
//     const uint32_t erase_len  = align_up(prog_len, FLASH_SECTOR_SIZE);

//     uint32_t ints = save_and_disable_interrupts();
//     flash_range_erase(LOG_OFFSET, erase_len);
//     flash_range_program(LOG_OFFSET, header_block, HEADER_STORE_LEN);

//     uint32_t written = 0;
//     uint32_t addr = LOG_OFFSET + HEADER_STORE_LEN;
//     while (written < data_bytes) {
//         uint8_t page[HEADER_STORE_LEN];
//         uint32_t chunk = (data_bytes - written >= HEADER_STORE_LEN) ? HEADER_STORE_LEN : (data_bytes - written);
//         memset(page, 0xFF, sizeof(page));
//         memcpy(page, ((const uint8_t *)cap_words) + written, chunk);
//         flash_range_program(addr, page, HEADER_STORE_LEN);
//         addr    += HEADER_STORE_LEN;
//         written += chunk;
//     }
//     restore_interrupts(ints);
// }

// void logic_erase_saved_capture(void) { //  erase the previous from flash
//     uint32_t ints = save_and_disable_interrupts();
//     flash_range_erase(LOG_OFFSET, LOG_REGION_SIZE);
//     restore_interrupts(ints);
//     printf("ERASED\n");
// }

// static bool read_config_from_flash(flash_config_t *out) {
//     const flash_config_t *stored =
//         (const flash_config_t *)(XIP_BASE + FLASH_CONFIG_ADDR);

//     if (stored->magic   != FLASH_CONFIG_MAGIC ||
//         stored->version != FLASH_CONFIG_VERSION) {
//         return false;  // no valid config yet
//     }

//     *out = *stored;
//     return true;
// }

// static void write_config_to_flash(const flash_config_t *cfg) {
//     // Flash must be erased per sector
//     uint8_t page[FLASH_CONFIG_SECTOR_SIZE];
//     memset(page, 0xFF, sizeof(page));
//     memcpy(page, cfg, sizeof(*cfg));

//     uint32_t ints = save_and_disable_interrupts();
//     flash_range_erase(FLASH_CONFIG_ADDR, FLASH_CONFIG_SECTOR_SIZE);
//     flash_range_program(FLASH_CONFIG_ADDR, page, FLASH_CONFIG_SECTOR_SIZE);
//     restore_interrupts(ints);
// }