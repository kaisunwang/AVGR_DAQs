/**
 * NeoPixel (WS2812) status LED on GPIO16
 *
 * - PIO state machine drives the one-wire protocol
 * - Supports solid color, single blink, and non-blocking multi-blink sequences
 * - Timer callback services blink state every 5 ms
 */

#include "neopixel.h"
#include "pico/time.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

/* ---------- Hardware config: GPIO16, PIO1 SM0 ---------- */
#define NEOPIXEL_PIO      pio1
#define NEOPIXEL_SM       0
#define NEOPIXEL_PIN      16
#define NEOPIXEL_IS_RGBW  false

/* Load ws2812 PIO program and configure state machine for given freq (Hz). */
static inline void ws2812_program_init(PIO pio, uint sm, uint offset,
                                       uint pin, float freq, bool rgbw) {
    /* Init the data pin as PIO-controlled output */
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

    /* Base config from generated ws2812.pio.h */
    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);

    /* WS2812: 24 (RGB) or 32 (RGBW) bits, MSB first, autopull */
    sm_config_set_out_shift(&c, false, true, rgbw ? 32 : 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    /* PIO bit clock = sys_clk / (freq * cycles_per_bit) */
    const float cycles_per_bit = ws2812_T1 + ws2812_T2 + ws2812_T3; // 10
    float div = (float)clock_get_hz(clk_sys) / (freq * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);

    /* Apply config and start SM */
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

/* WS2812 expects GRB order; pack into 24-bit value (shifted for autopull). */
static inline uint32_t pack_grb(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
}

static inline void neopixel_write_rgb_raw(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t grb = pack_grb(r, g, b);
    pio_sm_put_blocking(NEOPIXEL_PIO, NEOPIXEL_SM, grb << 8u);
}

static inline void neopixel_off_raw(void) {
    neopixel_write_rgb_raw(0, 0, 0);
}

/* State for non-blocking multi-blink: on_ms, off_ms, remaining count. */
typedef struct {
    bool     active;
    bool     led_on;
    uint8_t  r, g, b;
    uint32_t on_ms;
    uint32_t off_ms;
    uint32_t blinks_left;      // full on+off cycles remaining
    uint32_t next_change_ms;   // next time (ms since boot) to toggle
} neopixel_blink_state_t;

static neopixel_blink_state_t g_np_blink = {0};

/* Called periodically by timer. Toggle LED on/off according to schedule. */
void neopixel_blink_service(void)
{
    if (!g_np_blink.active) return;

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((int32_t)(now - g_np_blink.next_change_ms) < 0) {
        return; /* Not time to toggle yet */
    }

    if (!g_np_blink.led_on) {
        /* Turn LED on; schedule next toggle for on_ms later */
        neopixel_write_rgb_raw(g_np_blink.r, g_np_blink.g, g_np_blink.b);
        g_np_blink.led_on = true;
        g_np_blink.next_change_ms = now + g_np_blink.on_ms;
    } else {
        /* Turn LED off; one full blink done, decrement count */
        neopixel_off_raw();
        g_np_blink.led_on = false;

        if (g_np_blink.blinks_left > 0) {
            g_np_blink.blinks_left--;
        }

        if (g_np_blink.blinks_left == 0) {
            /* Sequence complete; stop blinking */
            g_np_blink.active = false;
        } else {
            g_np_blink.next_change_ms = now + g_np_blink.off_ms;
        }
    }
}

/* Repeating timer: fires every 5 ms to service blink state machine */
static repeating_timer_t g_np_timer;

/* PIO program offset (saved at init so it can be removed on deinit). */
static uint g_np_offset = 0;
static bool g_np_inited = false;

/* Timer callback: runs neopixel_blink_service(); return true to keep repeating */
static bool neopixel_blink_timer_cb(repeating_timer_t *rt) {
    neopixel_blink_service();
    return true;    // keep repeating
}

void neopixel_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    /* Cancel any ongoing blink; set solid color */
    g_np_blink.active = false;
    neopixel_write_rgb_raw(r, g, b);
}

void neopixel_off(void) {
    g_np_blink.active = false;
    neopixel_off_raw();
}

/* Start non-blocking multi-blink: 'count' cycles of (on_ms on, off_ms off).
 * Returns immediately; neopixel_blink_service() (via timer) does the toggling. */
void neopixel_blink_multiple_start(uint8_t r, uint8_t g, uint8_t b,
                                   uint32_t on_ms, uint32_t off_ms,
                                   uint32_t count)
{
    if (count == 0) {
        /* Stop any existing pattern */
        g_np_blink.active = false;
        neopixel_off();
        return;
    }

    g_np_blink.active      = true;
    g_np_blink.led_on      = false;
    g_np_blink.r           = r;
    g_np_blink.g           = g;
    g_np_blink.b           = b;
    g_np_blink.on_ms       = on_ms;
    g_np_blink.off_ms      = off_ms;
    g_np_blink.blinks_left = count;
    g_np_blink.next_change_ms = to_ms_since_boot(get_absolute_time());  /* Toggle ASAP */
}

/* Blocking: set color, hold for ms, then turn off. */
void neopixel_blink_once(uint8_t r, uint8_t g, uint8_t b, uint32_t ms) {
    neopixel_set_rgb(r, g, b);
    sleep_ms(ms);
    neopixel_off();
}

void neopixel_init(void) {
    /* Load PIO program, init state machine, configure GPIO16 */
    g_np_offset = pio_add_program(NEOPIXEL_PIO, &ws2812_program);
    ws2812_program_init(NEOPIXEL_PIO, NEOPIXEL_SM, g_np_offset,
                        NEOPIXEL_PIN, 800000.0f, NEOPIXEL_IS_RGBW);
    g_np_inited = true;

    /* -5 ms = repeat every 5 ms (negative = relative, not absolute) */
    add_repeating_timer_ms(-5, neopixel_blink_timer_cb, NULL, &g_np_timer);

    /* Dim orange blink during SD/config init (10k blinks = ~500 s if needed) */
    neopixel_blink_multiple_start(15, 2, 0, 50, 50, 10000);
}

/* Stop the blink timer (e.g. after config loaded, final LED set elsewhere). */
void neopixel_cancel_timer(void) {
    cancel_repeating_timer(&g_np_timer);
}

/* Fully release the NeoPixel: stop timer, blank the LED, disable the state
 * machine, and remove the PIO program. Safe to call if already deinited. */
void neopixel_deinit(void) {
    if (!g_np_inited) return;

    cancel_repeating_timer(&g_np_timer);
    g_np_blink.active = false;

    neopixel_off_raw();   // blank the LED before tearing down

    pio_sm_set_enabled(NEOPIXEL_PIO, NEOPIXEL_SM, false);
    pio_remove_program(NEOPIXEL_PIO, &ws2812_program, g_np_offset);

    g_np_inited = false;
}