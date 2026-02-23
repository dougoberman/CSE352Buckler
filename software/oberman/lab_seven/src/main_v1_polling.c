/*
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║         V1 — POLLING LED MODE CONTROLLER  (Deliberately Limited)        ║
 * ║         CSE 352/452 — Embedded Systems Lab 7                            ║
 * ║                                                                          ║
 * ║  Architecture:  Single while(1) loop — no threads, no interrupts        ║
 * ║                                                                          ║
 * ║  ┌───────────────────── THE EXPERIMENT ────────────────────────────┐    ║
 * ║  │ Change SLEEP_TIME_MS to 50, 200, and 500 and observe:           │    ║
 * ║  │                                                                  │    ║
 * ║  │   50 ms  → button feels very responsive, LEDs flicker too fast  │    ║
 * ║  │  200 ms  → mediocre at both: button sluggish, LEDs passable     │    ║
 * ║  │  500 ms  → LEDs slow and smooth, button laggy/misses presses    │    ║
 * ║  │                                                                  │    ║
 * ║  │ CONCLUSION: There is NO good value for SLEEP_TIME_MS because    │    ║
 * ║  │ the loop controls both button polling rate AND LED frame rate    │    ║
 * ║  │ with a single number.  They are fundamentally coupled.          │    ║
 * ║  │                                                                  │    ║
 * ║  │ V3 solves this by running button and LED logic in separate       │    ║
 * ║  │ threads, each sleeping at the rate appropriate for its job.      │    ║
 * ║  └──────────────────────────────────────────────────────────────────┘    ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * To build with this file:
 *   cp src/main_v1_polling.c src/main.c
 *   west build -b nrf52dk/nrf52832 -p always -- -DDTC_OVERLAY_FILE=nrf52dk_nrf52832.overlay
 *   west flash
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * EXPERIMENT KNOB  ← STUDENTS: CHANGE THIS VALUE
 *
 * Try:  50  (fast poll, fast LEDs — button great, animation bad)
 *       200 (medium — neither button nor LEDs feel good)
 *       500 (slow poll, slow LEDs — animation OK, button terrible)
 *
 * There is no correct value.  This is the demonstration.
 * ═══════════════════════════════════════════════════════════════════════════ */

#define SLEEP_TIME_MS  200   /* ← try 50, 200, 500 and observe the tradeoff  */

/* ═══════════════════════════════════════════════════════════════════════════
 * MODE DEFINITION  (identical across V1, V2, V3)
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef enum {
	MODE_OFF = 0,
	MODE_FLASH,
	MODE_CHASE,
	MODE_PINGPONG,
	MODE_BINARY,
	MODE_COUNT
} led_mode_t;

static const char * const mode_names[] = {
	"OFF", "FLASH", "CHASE", "PINGPONG", "BINARY",
};

/* ═══════════════════════════════════════════════════════════════════════════
 * HARDWARE BINDINGS  (identical across V1, V2, V3)
 * ═══════════════════════════════════════════════════════════════════════════ */

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec btn  = GPIO_DT_SPEC_GET(DT_ALIAS(sw0),  gpios);

static const struct gpio_dt_spec *leds[] = { &led0, &led1, &led2 };
#define NUM_LEDS ARRAY_SIZE(leds)

/* ═══════════════════════════════════════════════════════════════════════════
 * LED HELPER FUNCTIONS  (identical across V1, V2, V3)
 * ═══════════════════════════════════════════════════════════════════════════ */

static void leds_off(void)
{
	for (int i = 0; i < NUM_LEDS; i++) {
		gpio_pin_set_dt(leds[i], 0);
	}
}

static void leds_set_pattern(uint8_t pattern)
{
	for (int i = 0; i < NUM_LEDS; i++) {
		gpio_pin_set_dt(leds[i], (pattern >> i) & 1);
	}
}

/* ═══════════════════════════════════════════════════════════════════════════
 * main() — THE ENTIRE PROGRAM LIVES IN THIS ONE FUNCTION
 *
 * No threads, no interrupts, no message queues, no mutexes.
 * Just one loop doing everything.
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(void)
{
	printk("╔════════════════════════════════════╗\n");
	printk("║  CSE 352/452 — Lab 7               ║\n");
	printk("║  V1: Polling Design                 ║\n");
	printk("║  SLEEP_TIME_MS = %-4d              ║\n", SLEEP_TIME_MS);
	printk("╚════════════════════════════════════╝\n");

	/* ── Hardware init ───────────────────────────────────────────────────── */
	if (!gpio_is_ready_dt(&led0) || !gpio_is_ready_dt(&led1) ||
	    !gpio_is_ready_dt(&led2) || !gpio_is_ready_dt(&btn)) {
		printk("ERROR: GPIO device not ready!\n");
		return -1;
	}

	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&btn, GPIO_INPUT);

	/* ── State variables ─────────────────────────────────────────────────── */

	led_mode_t current_mode = MODE_OFF;

	/*
	 * Track the previous button state for edge detection.
	 * gpio_pin_get_dt() with GPIO_ACTIVE_LOW returns:
	 *   1 = button pressed  (GPIO physically LOW = "active")
	 *   0 = button released (GPIO physically HIGH = "inactive")
	 * Initial value 0 = released (the normal unpressed state).
	 */
	int last_btn_state = 0;

	uint32_t step = 0;   /* animation frame counter, reset on mode change    */

	/* PINGPONG sweep sequence (led indices): 0→1→2→1→0→1→2→1→…             */
	static const uint8_t pingpong_seq[] = { 0, 1, 2, 1 };

	/* ═══════════════════════════════════════════════════════════════════════
	 * MAIN LOOP
	 *
	 * This single loop reads the button AND drives the LEDs in the same
	 * iteration.  The k_msleep() at the bottom controls BOTH the button
	 * polling rate AND the LED animation speed simultaneously.
	 *
	 * That is the fundamental coupling problem.  It cannot be fixed by
	 * choosing a better SLEEP_TIME_MS — only by redesigning the architecture
	 * (see V3).
	 * ═══════════════════════════════════════════════════════════════════════ */

	while (1) {

		/* ── Poll the button ─────────────────────────────────────────────
		 * Read the current logical state of the button pin.
		 * We can only check as often as we loop — once every SLEEP_TIME_MS.
		 * Any press shorter than SLEEP_TIME_MS may be missed entirely.
		 */
		int btn_state = gpio_pin_get_dt(&btn);

		/*
		 * Edge detection: detect the rising edge of the logical signal,
		 * which corresponds to the button being PRESSED.
		 * (prev = released = 0, curr = pressed = 1)
		 */
		if (btn_state == 1 && last_btn_state == 0) {
			current_mode = (led_mode_t)((current_mode + 1) % MODE_COUNT);
			step = 0;   /* restart animation from frame 0 in new mode   */
			printk("[Poll] Mode -> %s\n", mode_names[current_mode]);
		}
		last_btn_state = btn_state;

		/* ── Drive LED pattern ───────────────────────────────────────────
		 * Produce one frame of the current mode's animation.
		 * The frame rate is 1000 / SLEEP_TIME_MS Hz — controlled by the
		 * same sleep that controls button polling.  Changing one changes both.
		 */
		switch (current_mode) {

		case MODE_OFF:
			leds_off();
			break;

		case MODE_FLASH:
			leds_set_pattern((step % 2 == 0) ? 0x07u : 0x00u);
			break;

		case MODE_CHASE:
			leds_set_pattern(1u << (step % 3));
			break;

		case MODE_PINGPONG:
			leds_set_pattern(
				1u << pingpong_seq[step % ARRAY_SIZE(pingpong_seq)]);
			break;

		case MODE_BINARY:
			leds_set_pattern((uint8_t)(step % 8));
			break;

		default:
			leds_off();
			break;
		}

		step++;

		/*
		 * ← THIS SLEEP IS THE PROBLEM.
		 *
		 * It controls BOTH:
		 *   • How often we check the button  (polling rate)
		 *   • How fast the LED animation runs (frame rate)
		 *
		 * They share the same number.  There is no value that optimizes both.
		 *
		 *   50 ms  → 20 fps LEDs (too fast), 20 Hz button poll (responsive)
		 *  200 ms  →  5 fps LEDs (OK),        5 Hz button poll (sluggish)
		 *  500 ms  →  2 fps LEDs (smooth),    2 Hz button poll (terrible)
		 *
		 * ← STUDENTS: Change SLEEP_TIME_MS above and observe this tradeoff.
		 */
		k_msleep(SLEEP_TIME_MS);
	}

	return 0;
}
