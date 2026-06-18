#pragma once

#include <stdint.h>

#include "hardware/i2c.h"

#include "sd_config.h"   // trigger_time_t

#define RTC_ADDR     0x6F   // MCP7940N address (1101111X, X = r/w)
#define REG_RTCSEC   0x00   // timekeeping registers start here (SEC, MIN, HOUR)

// ── helpers ──────────────────────────────────────────────────────────────────

static inline uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static inline uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

// ── set time ──────────────────────────────────────────────────────────────

/* Write hours/minutes/seconds and start the oscillator (ST bit), 24-hour mode. */
void mcp_set_time(i2c_inst_t *i2c, uint8_t hours, uint8_t minutes, uint8_t seconds);

// ------ turn on battery
void mcp_enable_battery(i2c_inst_t *i2c);

// ── get time ────────────────────────────────────────────────────────────

/* Read DAY/SEC/MIN/HOUR from the RTC and return them in a time_t.
   Date fields (year/month/day) are always zeroed. On an I2C error the
   returned struct is all zeros. */
trigger_time_t rtc_get_time(i2c_inst_t *i2c); //the struct is trigger_time_t but it is not necessarily trigger time

/* Check if the MCP7940N is running by reading the OSCRUN bit. */
bool mcp_is_running(i2c_inst_t *i2c);
