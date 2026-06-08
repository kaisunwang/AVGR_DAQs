/**
 * SD card configuration + capture logging.
 * SPI1 mapping: SCK=GP10, TX/MOSI=GP11, RX/MISO=GP12, CS=GP13
 */

#include "sd_config.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ff.h"
#include "f_util.h"
#include "hw_config.h"
#include "neopixel.h"
#include "sd_card.h"
#include "flash_log.h"

/* file prefix things*/
#define FILE_PREFIX_MAX_LEN  16
#define DEFAULT_FILE_PREFIX  "CAP_"

/* Path to the config file on the SD card. */
#define CONFIG_PATH  "0:config.txt"

static config_state_t g_config_state = CONFIG_STATE_NO_SD;


// ---- FatFs SPI hardware config (merged from hw_config.c) ----
static spi_t spis[] = {
    {
        .hw_inst = spi1,
        .miso_gpio = 12,             // RX (MISO)
        .mosi_gpio = 11,             // TX (MOSI)
        .sck_gpio = 10,
        .baud_rate = 10000 * 1000,   // 10 MHz
        
        .set_drive_strength = false,
        .mosi_gpio_drive_strength = GPIO_DRIVE_STRENGTH_4MA,
        .sck_gpio_drive_strength  = GPIO_DRIVE_STRENGTH_4MA,
    }
};

static sd_card_t sd_cards[] = {
    {
        .pcName = "0:",     // Logical drive name for FatFS
        .spi    = &spis[0], // SPI used by this card

        .ss_gpio = 13,      // SD card CS pin (GPIO number)

        // We’re *not* using a card-detect switch
        .use_card_detect   = false,
        .card_detect_gpio  = 0,   // ignored when use_card_detect == false
        .card_detected_true = 0,  // ignored when use_card_detect == false

        .set_drive_strength       = false,
        .ss_gpio_drive_strength   = GPIO_DRIVE_STRENGTH_4MA,
    }
};

size_t sd_get_num(void) {
    return sizeof(sd_cards) / sizeof(sd_cards[0]);
}

sd_card_t *sd_get_by_num(size_t num) {
    if (num < sd_get_num()) {
        return &sd_cards[num];
    }
    return NULL;
}

size_t spi_get_num(void) {
    return sizeof(spis) / sizeof(spis[0]);
}

spi_t *spi_get_by_num(size_t num) {
    if (num < spi_get_num()) {
        return &spis[num];
    }
    return NULL;
}

// ---- SD capture write variables ----
static FATFS g_fs;
static bool g_mounted = false;
static bool g_init_tried = false;
static uint32_t g_file_index = 0;

#define RUN_DIR_NAME_MAX_LEN  16
static uint32_t g_run_index = 0;
static char     g_run_dir_name[RUN_DIR_NAME_MAX_LEN] = {0};  // e.g. "RUN_00012"
static uint32_t g_target_sample_hz = DEFAULT_SAMPLE_HZ;
// Per-peripheral capture prefixes. Index i (0-based) holds the value of
// PREFIX(i+1) from config.txt, e.g. g_prefixes[0] <- "PREFIX1 = ...".
static char     g_prefixes[MAX_PREFIXES][FILE_PREFIX_MAX_LEN];
static bool     g_prefixes_inited = false;
// Number of peripherals; set from config.txt "N = _", default DEFAULT_N_PERIPHS.
static uint8_t  g_num_periphs = DEFAULT_N_PERIPHS;



bool sd_init(void) {
    if (g_mounted) {
        return true;
    }
    // if (g_init_tried) {
    //     return false;
    // }

    // g_init_tried = true;

    if (!sd_init_driver()) {
        printf("SD: sd_init_driver() failed.\n");
        return false;
    }

    FRESULT fr = f_mount(&g_fs, "0:", 1);
    if (fr != FR_OK) {
        printf("SD: f_mount failed (%s)\n", FRESULT_str(fr));
        return false;
    }

    g_mounted = true;
    printf("SD: mounted OK.\n");
    return true;
}

static void trim_line(char *s) {
    // Strip trailing \r\n and spaces
    size_t len = strlen(s);
    while (len > 0 && (s[len-1] == '\r' || s[len-1] == '\n' || s[len-1] == ' ' || s[len-1] == '\t')) {
        s[--len] = '\0';
    }
}

// Fill every prefix slot with its default ("cap1", "cap2", ... 1-indexed).
static void set_default_prefixes(void) {
    for (uint8_t i = 0; i < MAX_PREFIXES; i++) {
        snprintf(g_prefixes[i], FILE_PREFIX_MAX_LEN, "%s%u",
                 DEFAULT_FILE_PREFIX, (unsigned)(i + 1));
    }
    g_prefixes_inited = true;
}

uint8_t sd_get_num_periphs(void) {
    return g_num_periphs;
}

static config_state_t load_config_from_sd(void) {
    // Start every prefix at its default ("cap1".."capN") and reset the
    // peripheral count to its default; config.txt overrides both.
    set_default_prefixes();
    g_num_periphs = DEFAULT_N_PERIPHS;

    // Ensure SD init was attempted
    sd_init();

    if (!g_mounted) {
        printf("Config: SD not mounted; using defaults / flash.\n");
        // LED blue: will write to flash
        neopixel_set_rgb(0, 0, 100);
        g_config_state = CONFIG_STATE_NO_SD;
        return g_config_state;
    }

    FIL fil;
    FRESULT fr = f_open(&fil, CONFIG_PATH, FA_READ);

    if (fr == FR_NO_FILE) {
        // SD is there, but no config.txt -> ORANGE state
        printf("Config: %s not found on SD (FR=%d).\n", CONFIG_PATH, fr);
        //**removed fallback to flash config */
        // Orange LED: SD OK but no config file
        neopixel_set_rgb(100, 35, 0);
        g_config_state = CONFIG_STATE_NO_FILE;
        return g_config_state;
    } else if (fr != FR_OK) {
        // Some other SD error while opening config.txt
        printf("Config: could not open %s (FR=%d); treating as SD failure.\n", CONFIG_PATH, fr);
        neopixel_set_rgb(0, 0, 100);
        g_config_state = CONFIG_STATE_NO_SD;
        return g_config_state;
    }

    // --- SD config file is open here ---
    // Parse "N = _" (peripheral count) and "PREFIXn = value" (n is 1-based).
    char line[64];
    while (f_gets(line, sizeof(line), &fil)) {
        trim_line(line);

        // "N = 3": set the peripheral count (1..MAX_PREFIXES).
        if (line[0] == 'N') {
            const char *q = line + 1;
            while (*q == ' ' || *q == '\t') q++;
            if (*q == '=') {
                long n = strtol(q + 1, NULL, 10);
                if (n >= 1 && n <= MAX_PREFIXES) {
                    g_num_periphs = (uint8_t)n;
                    printf("Config: N=%u\n", g_num_periphs);
                } else {
                    printf("Config: N out of range; using default %u\n", g_num_periphs);
                }
            }
            continue;
        }

        if (strncmp(line, "PREFIX", 6) != 0) continue;

        // Read the 1-based index right after "PREFIX".
        const char *p = line + 6;
        char *endptr = NULL;
        long idx1 = strtol(p, &endptr, 10);
        if (endptr == p || idx1 < 1 || idx1 > MAX_PREFIXES) {
            continue;  // not a valid PREFIXn
        }

        // Skip optional spaces, require '=', then skip spaces before the value.
        const char *q = endptr;
        while (*q == ' ' || *q == '\t') q++;
        if (*q != '=') continue;
        q++;
        while (*q == ' ' || *q == '\t') q++;

        // Copy the value into the matching slot (PREFIXn -> index n-1).
        size_t len = strlen(q);
        if (len >= FILE_PREFIX_MAX_LEN) len = FILE_PREFIX_MAX_LEN - 1;
        memcpy(g_prefixes[idx1 - 1], q, len);
        g_prefixes[idx1 - 1][len] = '\0';
        printf("Config: PREFIX%ld=\"%s\"\n", idx1, g_prefixes[idx1 - 1]);
    }

    f_close(&fil);

    // SD present and config parsed: green.
    neopixel_set_rgb(0, 100, 0);
    g_config_state = CONFIG_STATE_MATCH;

    return g_config_state;
}

bool write_capture_to_sd(const uint8_t *buf, uint32_t len, uint32_t sample_hz, const trigger_time_t *trigger_time, uint8_t daq_num, uint8_t cap_cnt) {
    
    if (!buf || len == 0) {
        printf("SD: invalid write buffer.\n");
        neopixel_set_rgb(100, 0, 0); //red
        return false;
    }

    if (!sd_init()) {
        printf("SD: sd_init() failed.\n");
        neopixel_set_rgb(100, 0, 0); //red
        
        // STORE TO FLASH****
        // logic_save_capture_to_flash_raw(buf, len, sample_hz, trigger_time, daq_num);
        return false; 
    }
    neopixel_set_rgb(0, 0, 100);  // blue while writing

    // Make sure prefixes have at least their defaults (in case load_config
    // was never called before this write).
    if (!g_prefixes_inited) set_default_prefixes();

    // Select this peripheral's prefix; guard against an out-of-range daq_num.
    const char *prefix = (daq_num < g_num_periphs) ? g_prefixes[daq_num]
                                                   : DEFAULT_FILE_PREFIX;

    char filename[64];
    // Filename: RUN_xxxxx/<prefix>_<cap_cnt>_DD_HH_MM_SS.bin
    if (trigger_time && g_run_dir_name[0]) {
        snprintf(filename, sizeof(filename), "0:%s/%s_%02u_%02u_%02u_%02u_%02u.bin",
                 g_run_dir_name,
                 prefix,
                 cap_cnt,
                 trigger_time->day,
                 trigger_time->hour,
                 trigger_time->min,
                 trigger_time->sec);
    } else {
        snprintf(filename, sizeof(filename), "0:cap_%04lu.bin",
                 (unsigned long)g_file_index);
    }
    printf("SD: writing to file %s\n", filename);

    FIL fil;
    UINT written = 0;

    FRESULT fr = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS);
    printf("SD: f_open(%s) result: %s\n", filename, FRESULT_str(fr));
    if (fr != FR_OK) {
        printf("SD: f_open(%s) failed (%s)\n", filename, FRESULT_str(fr));
        neopixel_set_rgb(100, 0, 0);
        // logic_save_capture_to_flash_raw(buf, len, sample_hz, trigger_time, daq_num);
        return false;
    }

    fr = f_write(&fil, buf, len, &written);
    printf("SD: f_write(%s) wrote %lu bytes\n", filename, written);
    
    if (fr != FR_OK || written != len) {
        printf("SD: f_write failed (%u/%lu, %s)\n",
               written, (unsigned long)len, FRESULT_str(fr));
        (void)f_close(&fil);
        neopixel_set_rgb(100, 0, 0);
        // logic_save_capture_to_flash_raw(buf, len, sample_hz, trigger_time, daq_num);
        return false;
    }

    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("SD: f_close failed but should be chill (%s)\n", FRESULT_str(fr));
        neopixel_set_rgb(100, 0, 0);
        return false;
    }

    printf("SD: wrote %lu bytes to %s\n", (unsigned long)len, filename);
    g_file_index++;
    // neopixel_set_rgb(0, 100, 0);  // green on success
    return true;
}

void sd_unmount(void) {
    if (!g_mounted) return;
    f_unmount("0:");
    g_mounted = false;
    printf("SD: unmounted.\n");
}


/*new functions*/
void init_run_directory(void) {
    if (!g_mounted) {
        printf("RunDir: SD not mounted; no run directory.\n");
        g_run_dir_name[0] = '\0';
        g_run_index = 0;
        g_file_index = 0;
        return;
    }

    config_state_t config_state = load_config_from_sd();    

    DIR dir;
    FILINFO fno;
    FRESULT fr;

    fr = f_opendir(&dir, "0:");  // root of the SD card
    if (fr != FR_OK) {
        printf("RunDir: f_opendir failed (FR=%d)\n", fr);
        g_run_dir_name[0] = '\0';
        g_run_index = 0;
        g_file_index = 0;
        return;
    }

    uint32_t max_run_idx = 0;
    bool found_any = false;
    char digits[6];  // 5 digits + NUL

    const char *RUN_PREFIX = "RUN_";
    size_t rplen = strlen(RUN_PREFIX);

    for (;;) {
        fr = f_readdir(&dir, &fno);
        if (fr != FR_OK || fno.fname[0] == 0) {
            // error or end of directory
            break;
        }

        // Only consider directories
        if (!(fno.fattrib & AM_DIR)) continue;

        const char *name = fno.fname;
        size_t nlen = strlen(name);
        if (nlen < rplen + 5) continue; // need at least "RUN_00000"

        if (strncmp(name, RUN_PREFIX, rplen) != 0) continue;

        const char *p_digits = name + rplen;
        memcpy(digits, p_digits, 5);
        digits[5] = '\0';

        char *endptr = NULL;
        long idx = strtol(digits, &endptr, 10);
        if (*endptr != '\0' || idx < 0) {
            continue;
        }

        if (!found_any || (uint32_t)idx > max_run_idx) {
            max_run_idx = (uint32_t)idx;
            found_any = true;
        }
    }

    f_closedir(&dir);

    if (found_any) {
        g_run_index = max_run_idx + 1;
    } else {
        g_run_index = 0;
    }

    snprintf(g_run_dir_name, sizeof(g_run_dir_name),
             "RUN_%05lu", (unsigned long)g_run_index); // writes RUN_ + g_run_dinex into the memory allocated to g_rn_dir_name

    printf("RunDir: creating %s\n", g_run_dir_name);

    fr = f_mkdir(g_run_dir_name);
    if (fr != FR_OK && fr != FR_EXIST) {
        printf("RunDir: f_mkdir('%s') failed (FR=%d)\n", g_run_dir_name, fr);
        // If this fails, we fall back to root
        g_run_dir_name[0] = '\0';
    }

    // start file index at zero for this run
    g_file_index = 0;

    // Write metadata file into this run directory
    if (g_run_dir_name[0]) {
        write_run_metadata_file();
    }
}


static void write_run_metadata_file(void) {
    if (!g_mounted || g_run_dir_name[0] == '\0') {
        return;
    }

    FIL fil;
    char path[64];
    FRESULT fr;

    // Path: RUN_00012/meta.txt
    snprintf(path, sizeof(path), "%s/meta.txt", g_run_dir_name);

    fr = f_open(&fil, path, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("Meta: f_open('%s') failed (FR=%d)\n", path, fr);
        return;
    }

    char line[64];

    // Basic run info
    snprintf(line, sizeof(line), "RUN_INDEX=%lu\n", (unsigned long)g_run_index);
    f_puts(line, &fil);

    snprintf(line, sizeof(line), "SAMPLE_HZ=%u\n", DEFAULT_SAMPLE_HZ);
    f_puts(line, &fil);

    snprintf(line, sizeof(line), "N=%u\n", (unsigned)g_num_periphs);
    f_puts(line, &fil);
    for (uint8_t i = 0; i < g_num_periphs; i++) {
        snprintf(line, sizeof(line), "PREFIX%u=%s\n",
                 (unsigned)(i + 1), g_prefixes[i]);
        f_puts(line, &fil);
    }

    // snprintf(line, sizeof(line), "DATA_DEPTH=%u\n", CAPTURE_N_SAMPLES);
    // f_puts(line, &fil);

    // You could add more here: build version, date stamp, etc.

    f_close(&fil);
    printf("Meta: wrote %s\n", path);
}