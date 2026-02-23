/*
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║         V2 — BAD ISR LED MODE CONTROLLER  (Deliberately Broken)         ║
 * ║         CSE 352/452 — Embedded Systems Lab 7                            ║
 * ║                                                                          ║
 * ║  Architecture:  GPIO interrupt + while(1) LED loop in main()            ║
 * ║                                                                          ║
 * ║  V2 fixes V1's coupling problem by using a GPIO interrupt.              ║
 * ║  The LED loop runs freely at its own rate.  The button interrupt fires  ║
 * ║  independently.  Timing is no longer coupled — that part works!         ║
 * ║                                                                          ║
 * ║  BUT V2 introduces three new bugs, marked below with BUG 1/2/3:        ║
 * ║                                                                          ║
 * ║  BUG 1 — No debounce                                                    ║
 * ║    Mechanical switches bounce: a single human press causes the GPIO     ║
 * ║    line to oscillate 3–10 times before settling.  Each bounce fires     ║
 * ║    the ISR.  Without debounce, one press skips 3–10 modes.             ║
 * ║                                                                          ║
 * ║  BUG 2 — printk() in ISR context                                        ║
 * ║    printk() blocks waiting for the UART TX buffer to drain.  An ISR     ║
 * ║    must NEVER block.  On Zephyr this can cause a kernel panic or        ║
 * ║    silently corrupt output.  Output may appear garbled or interleaved.  ║
 * ║                                                                          ║
 * ║  BUG 3 — Shared variable with no mutex                                  ║
 * ║    current_mode is written by the ISR and read by the main loop.        ║
 * ║    volatile prevents register caching, but does NOT prevent torn        ║
 * ║    reads/writes or compiler reordering — a mutex is required.           ║
 * ║                                                                          ║
 * ║  All three bugs are fixed in V3 (main.c).                               ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * To build with this file:
 *   cp src/main_v2_isr_bad.c src/main.c
 *   west build -b nrf52dk/nrf52832 -p always -- -DDTC_OVERLAY_FILE=nrf52dk_nrf52832.overlay
 *   west flash
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define LED_STEP_MS  200   /* LED animation frame period in ms               */

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
 * SHARED STATE — BUG 3: no mutex
 *
 * volatile tells the compiler: "this variable may change at any time;
 * always reload it from memory rather than using a cached register copy."
 *
 * That IS necessary (without it the compiler may hoist the read out of
 * the loop entirely), but it is NOT sufficient for correctness:
 *   • volatile does not prevent the compiler from reordering memory accesses
 *     around the read or write of this variable.
 *   • volatile does not prevent a torn read/write if the type is wider than
 *     one memory bus transaction (not an issue for 32-bit ARM + 32-bit enum,
 *     but IS an issue on 8-bit AVR or for 64-bit values).
 *
 * A proper fix requires a mutex (see V3) or an atomic type.
 * ═══════════════════════════════════════════════════════════════════════════ */

static volatile led_mode_t current_mode = MODE_OFF;   /* BUG 3: no mutex    */

/* gpio_callback must persist after gpio_add_callback() — file scope only.   */
static struct gpio_callback btn_cb_data;

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
 * ISR — Button GPIO Interrupt Handler  (deliberately wrong)
 *
 * This runs in INTERRUPT CONTEXT.  Every line below breaks a rule.
 * The correct version (V3) has exactly 3 lines of real code here.
 *
 * Signature is FIXED by Zephyr's GPIO API — do not change it.
 * ═══════════════════════════════════════════════════════════════════════════ */

static void button_isr(const struct device *dev,
		       struct gpio_callback *cb,
		       uint32_t pins)
{
	/*
	 * BUG 1 — NO DEBOUNCE
	 *
	 * A mechanical button press causes the GPIO line to bounce (oscillate
	 * rapidly) for roughly 1–20 ms before settling.  Each bounce edge fires
	 * this ISR again.  Without any debounce filter, a single human press
	 * will advance current_mode multiple times — typically 3 to 10 times.
	 *
	 * You can observe this: press the button once and watch the mode jump
	 * by more than one step (or cycle all the way around).
	 *
	 * FIX (V3): The ISR enqueues a timestamped event.  The Button Thread
	 * discards events that arrive within 250 ms of the last accepted one.
	 *
	 * BUG 3 (also here): writing current_mode without a mutex.
	 */
	current_mode = (led_mode_t)((current_mode + 1) % MODE_COUNT);  /* BUG 1+3 */

	/*
	 * BUG 2 — printk() IN ISR CONTEXT
	 *
	 * printk() is NOT async-signal-safe.  It needs to acquire an internal
	 * lock and may block waiting for space in the UART TX ring buffer.
	 * An ISR MUST NOT block — doing so prevents the CPU from servicing any
	 * other interrupt, can deadlock the kernel, and may trigger a watchdog
	 * reset or assertion failure.
	 *
	 * On Zephyr, printk() from an ISR will often appear to "work" during
	 * light testing but fail under load (e.g., when the UART is busy
	 * printing the LED task's output at the same time).  The output may be
	 * garbled or the board may hang.
	 *
	 * FIX (V3): printk() is called only from the Button Thread, never
	 * from the ISR.
	 */
	printk("[BAD ISR] Mode -> %s\n", mode_names[current_mode]);   /* BUG 2  */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * main() — Hardware init + LED animation loop
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(void)
{
	printk("╔════════════════════════════════════╗\n");
	printk("║  CSE 352/452 — Lab 7               ║\n");
	printk("║  V2: Bad ISR Design                 ║\n");
	printk("║  Watch for multi-mode skips!        ║\n");
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

	/*
	 * Interrupt configuration — this part is done correctly.
	 * The bugs in V2 are inside the ISR callback above, not in the setup.
	 */
	gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&btn_cb_data, button_isr, BIT(btn.pin));
	gpio_add_callback(btn.port, &btn_cb_data);

	printk("Hardware initialized. ISR active. Observe bounce...\n");

	/* ── Animation state ─────────────────────────────────────────────────── */

	uint32_t step = 0;
	static const uint8_t pingpong_seq[] = { 0, 1, 2, 1 };

	/* ═══════════════════════════════════════════════════════════════════════
	 * LED ANIMATION LOOP
	 *
	 * V2 improvement over V1: the LED loop runs at its own fixed rate.
	 * Button presses are handled by the ISR and do not affect frame timing.
	 * This solves V1's coupling problem.
	 *
	 * But BUG 3 note: current_mode is read here without a mutex.
	 * On 32-bit ARM, reading a 32-bit aligned enum is naturally atomic at
	 * the hardware level, so you likely won't see a torn read in practice.
	 * However:
	 *   (a) volatile does not prevent the compiler from reordering the read
	 *       relative to other memory operations.
	 *   (b) The correctness guarantee disappears on platforms where the type
	 *       requires multiple bus transactions (e.g., 64-bit values on 8-bit
	 *       MCUs).
	 *   (c) Using volatile shared state is undefined behavior in C — the
	 *       compiler is allowed to assume no concurrent writes.
	 * A mutex (V3) is the correct and portable fix.
	 * ═══════════════════════════════════════════════════════════════════════ */

	while (1) {

		led_mode_t mode = current_mode;   /* BUG 3: unsafe read, no mutex  */

		switch (mode) {

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
		k_msleep(LED_STEP_MS);
	}

	return 0;
}
