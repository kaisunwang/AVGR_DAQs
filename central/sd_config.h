#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "../peripheral/capture.h"

// ---------- Peripherals / capture prefixes ----------
// Hard upper bound on the number of prefixes/peripherals supported.
#define MAX_PREFIXES  8
// Peripheral count used when config.txt has no "N = _" line.
#define DEFAULT_N_PERIPHS  MAX_PREFIXES

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
} trigger_time_t;

typedef enum {
    CONFIG_STATE_NO_SD,          // SD not mounted / unusable
    CONFIG_STATE_NO_FILE,        // SD OK, but no config.txt
    CONFIG_STATE_MATCH,          // SD config matches flash config
    CONFIG_STATE_NEW_OR_CHANGED  // no flash config OR mismatch with SD config
} config_state_t;




// typedef struct {
//     // uint32_t magic;         // 'LAC1' = 0x3143414C
//     // uint16_t version;       // 1
//     uint16_t hdr_len;       // sizeof(header)
//     uint32_t sample_count;  // logical samples per pin
//     uint32_t pin_base;
//     uint32_t pin_count;
//     uint32_t sample_hz;     // effective sample rate
//     uint32_t trigger_pin;
//     uint32_t trigger_level; // 0/1
//     uint32_t word_count;    // 32-bit words stored following header
//     uint32_t reserved0;
// } logic_log_header_t;

/* Initialize SD driver and mount "0:". Returns true on success. */
bool sd_init(void); 

// static void trim_line(char *s) {
//     // Strip trailing \r\n and spaces
//     size_t len = strlen(s);
//     while (len > 0 && (s[len-1] == '\r' || s[len-1] == '\n' || s[len-1] == ' ' || s[len-1] == '\t')) {
//         s[--len] = '\0';
//     }
// }

/* Load config from SD*/
static config_state_t load_config_from_sd(void);

/* Write one capture buffer to "0:cap_day_hour_min_sec.bin". */
bool write_capture_to_sd(const uint8_t *buf, uint32_t len,uint32_t sample_hz, const trigger_time_t *trigger_time, uint8_t daq_num, uint8_t cap_cnt);

/* Unmount "0:" and release the SD card. Call before halting or reset. */
void sd_unmount(void);

/* Number of peripherals, from config.txt "N = _" (default DEFAULT_N_PERIPHS). */
uint8_t sd_get_num_periphs(void);


void init_run_directory(void);
// static void check_prefix_against_latest_run(void);
// static void init_sd_capture_index_from_existing_files(void);
static void write_run_metadata_file(void);
