#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Open a new numbered capture file (e.g. "cap_0001.bin") on the SD card.
 * Returns true on success. Must be called before 
 log_write(). */
bool flash_log_open_new(void);

/* Append 'len' bytes from 'data' to the currently open capture file.
 * Returns true on success. */
bool flash_log_write(const uint8_t *data, uint32_t len);

void logic_erase_saved_capture(void);

/* Check if there is a capture in flash*/
// bool logic_has_saved_capture(void);

/* Save capture to Flash*/
// void logic_save_capture_to_flash_raw(const uint32_t *cap_words,
//     uint32_t sample_count,
//     uint32_t word_count,
//     uint32_t pin_base,
//     uint32_t pin_count,
//     uint32_t sample_hz,
//     uint32_t trigger_pin,
//     uint32_t trigger_level);

/* Flush and close the current capture file. */
void flash_log_close(void);

/* Return the index of the last file written (increments each open). */
uint32_t flash_log_get_index(void);

/* Load from Flash*/
// bool read_config_from_flash(flash_config_t *out);

/* Save to Flash*/
// void write_config_to_flash(const flash_config_t *cfg);
