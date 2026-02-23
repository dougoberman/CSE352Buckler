/*
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║         V3 — EVENT-DRIVEN LED MODE CONTROLLER  (Student Skeleton)        ║
 * ║         CSE 352/452 — Embedded Systems Lab 7                            ║
 * ║                                                                          ║
 * ║  Architecture:                                                           ║
 * ║    ISR → k_msgq → Button Thread → (k_mutex) → LED Thread               ║
 * ║                                                                          ║
 * ║  Your job: fill in the four TODO sections below.                        ║
 * ║  The file compiles and flashes as-is, but the button does nothing       ║
 * ║  until you complete all four parts.                                      ║
 * ║                                                                          ║
 * ║  Compare to:                                                             ║
 * ║    V1 (main_v1_polling.c)  — coupled timing, no interrupts              ║
 * ║    V2 (main_v2_isr_bad.c)  — bounce, unsafe printk, no mutex            ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * CONSTANTS
 * Named constants so magic numbers never appear in logic code.
 * ═══════════════════════════════════════════════════════════════════════════ */

#define LED_STEP_MS   200   /* milliseconds per animation frame              */
#define DEBOUNCE_MS   250   /* minimum ms between two accepted button presses */

/* ═══════════════════════════════════════════════════════════════════════════
 * MODE DEFINITION
 *
 * The state machine that governs LED behavior.  Shared across V1, V2, V3.
 *
 * MODE_COUNT is always last — its integer value equals the number of modes,
 * so "(mode + 1) % MODE_COUNT" naturally wraps back to MODE_OFF.
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef enum {
	MODE_OFF = 0,
	MODE_FLASH,
	MODE_CHASE,
	MODE_PINGPONG,
	MODE_BINARY,
	MODE_COUNT          /* sentinel — always keep this last                   */
} led_mode_t;

/* Human-readable names for printk output. Must stay in sync with the enum. */
static const char * const mode_names[] = {
	"OFF",
	"FLASH",
	"CHASE",
	"PINGPONG",
	"BINARY",
};

/* ═══════════════════════════════════════════════════════════════════════════
 * HARDWARE BINDINGS
 *
 * GPIO_DT_SPEC_GET(DT_ALIAS(name), gpios) fetches the pin number, port,
 * and active-level flags directly from the device tree.  The overlay file
 * maps alias names to the Buckler Rev C physical pins.
 *
 * This means the C code has ZERO hard-coded pin numbers — the DT is the
 * single source of truth for hardware wiring.
 * ═══════════════════════════════════════════════════════════════════════════ */

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec btn  = GPIO_DT_SPEC_GET(DT_ALIAS(sw0),  gpios);

/* Convenience pointer array so helpers can loop without naming each LED.    */
static const struct gpio_dt_spec *leds[] = { &led0, &led1, &led2 };
#define NUM_LEDS ARRAY_SIZE(leds)

/* ═══════════════════════════════════════════════════════════════════════════
 * LED HELPER FUNCTIONS
 *
 * ALL LED writes in this file go through these two helpers.  No direct
 * gpio_pin_set_dt() calls are scattered around the animation logic.
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Turn every LED off (set all to logical 0 = inactive). */
static void leds_off(void)
{
	for (int i = 0; i < NUM_LEDS; i++) {
		gpio_pin_set_dt(leds[i], 0);
	}
}

/**
 * Drive LEDs from a 3-bit pattern.
 *   bit 0 → led0 (Red)
 *   bit 1 → led1 (Yellow)
 *   bit 2 → led2 (Blue)
 *
 * Example: leds_set_pattern(0b101) = 5  lights Red and Blue, Yellow off.
 * Zephyr handles the GPIO_ACTIVE_LOW inversion automatically.
 */
static void leds_set_pattern(uint8_t pattern)
{
	for (int i = 0; i < NUM_LEDS; i++) {
		gpio_pin_set_dt(leds[i], (pattern >> i) & 1);
	}
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SHARED STATE  (always access under mode_mutex)
 *
 * current_mode is written by the Button Thread and read by the LED Thread.
 * The mutex ensures neither thread sees a partially updated value.
 *
 * K_MUTEX_DEFINE is a compile-time macro — the mutex object exists at link
 * time.  k_mutex_lock / k_mutex_unlock are runtime function calls.
 *
 * NOTE: do NOT write "static K_MUTEX_DEFINE(...)".  The macro expands
 * through STRUCT_SECTION_ITERABLE which already emits "static" internally.
 * Writing it again causes a "duplicate 'static'" compile error.
 * Same rule applies to K_MSGQ_DEFINE and K_THREAD_DEFINE below.
 * ═══════════════════════════════════════════════════════════════════════════ */

static led_mode_t current_mode = MODE_OFF;
K_MUTEX_DEFINE(mode_mutex);

/* ═══════════════════════════════════════════════════════════════════════════
 * EVENT TYPE AND MESSAGE QUEUE
 *
 * The ISR cannot safely call printk, lock a mutex, or do debounce.
 * Instead it drops a timestamped event into this queue.  The Button Thread
 * dequeues events and decides (with debounce logic) whether to act.
 *
 * K_MSGQ_DEFINE parameters:
 *   name      — symbol name for the queue object
 *   msg_size  — bytes per message (sizeof our event struct)
 *   max_msgs  — queue depth (8 handles bursts during slow thread service)
 *   align     — byte alignment (4 is fine for a struct with one int64_t)
 * ═══════════════════════════════════════════════════════════════════════════ */

struct button_event {
	int64_t timestamp_ms;   /* k_uptime_get() captured at ISR fire time      */
};

K_MSGQ_DEFINE(btn_queue, sizeof(struct button_event), 8, 4);

/* ═══════════════════════════════════════════════════════════════════════════
 * GPIO CALLBACK OBJECT
 *
 * Zephyr requires the gpio_callback struct to persist after
 * gpio_add_callback().  Declare at file scope (static storage duration) so
 * it is never deallocated.  Never put it on the stack.
 * ═══════════════════════════════════════════════════════════════════════════ */

static struct gpio_callback btn_cb_data;

/* ═══════════════════════════════════════════════════════════════════════════
 * ISR — Button GPIO Interrupt Handler
 *
 * This runs in INTERRUPT CONTEXT.  The rules are strict:
 *   ✓  Read the uptime clock (async-signal-safe kernel call)
 *   ✓  Enqueue with K_NO_WAIT (non-blocking — ISR must never block)
 *   ✗  No printk()     — can block waiting for UART TX (see V2 for why)
 *   ✗  No k_mutex_lock — can sleep, which is illegal in ISR context
 *   ✗  No debounce     — no sleeping allowed; leave it to the thread
 *
 * The function signature is FIXED by Zephyr's GPIO API.  Do not change it.
 * ═══════════════════════════════════════════════════════════════════════════ */

static void button_isr(const struct device *dev,
		       struct gpio_callback *cb,
		       uint32_t pins)
{
	/*
	 * ── TODO 1 ───────────────────────────────────────────────────────────
	 * Fill in the ISR body.
	 *
	 * This function fires every time the button GPIO edge is detected.
	 * Your job is to record WHEN it fired and hand that information off
	 * to the Button Thread for processing.
	 *
	 * Allowed in ISR context:
	 *   ✓  k_uptime_get()              — reads the system clock, non-blocking
	 *   ✓  k_msgq_put(..., K_NO_WAIT)  — puts to queue without blocking
	 *
	 * NOT allowed in ISR context (do not add these):
	 *   ✗  printk()       — may block waiting for UART space (V2 Bug 2)
	 *   ✗  k_mutex_lock() — may sleep; sleeping in an ISR crashes the system
	 *   ✗  debounce math  — requires sleeping; belongs in the thread
	 *
	 * Steps:
	 *   1. Declare a local struct button_event variable named evt.
	 *   2. Set evt.timestamp_ms = k_uptime_get().
	 *   3. Call k_msgq_put(&btn_queue, &evt, K_NO_WAIT).
	 *      Always use K_NO_WAIT here — never K_FOREVER in an ISR.
	 * ─────────────────────────────────────────────────────────────────── */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * BUTTON THREAD
 *
 * Responsibilities:
 *   1. Block on the message queue — sleeps with zero CPU use when idle.
 *   2. Debounce — ignore events closer than DEBOUNCE_MS to the last accepted
 *      press (filters out mechanical bounce that V2 suffers from).
 *   3. Advance the mode — write current_mode under mutex protection.
 *   4. Print the new mode — safe here because we are in thread context,
 *      not ISR context (the V2 mistake).
 *
 * The three void* parameters are unused; ARG_UNUSED() silences the warning.
 * ═══════════════════════════════════════════════════════════════════════════ */

static void button_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct button_event evt;
	int64_t last_press_ms = 0;   /* uptime of the last accepted press        */

	while (1) {
		/*
		 * Block here until an event arrives.  While we wait, the
		 * scheduler runs other threads (e.g. the LED thread animates
		 * freely).  This is the core benefit of event-driven design:
		 * waiting costs nothing and does not hold up unrelated work.
		 */
		k_msgq_get(&btn_queue, &evt, K_FOREVER);

		/* ── TODO 2 ────────────────────────────────────────────────────
		 * Implement debounce.
		 *
		 * A mechanical button press bounces: the GPIO can fire 3–10 ISR
		 * events within a few milliseconds of a single human press.
		 * Without filtering, one press advances the mode many times —
		 * exactly what you observed in V2.
		 *
		 * You have:
		 *   evt.timestamp_ms  — when this event was captured (ms uptime)
		 *   last_press_ms     — when the last accepted press was captured
		 *   DEBOUNCE_MS       — the minimum gap (250 ms) between valid presses
		 *
		 * Logic:
		 *   - If (evt.timestamp_ms - last_press_ms) < DEBOUNCE_MS:
		 *       this event is a bounce — discard it with 'continue'.
		 *   - Otherwise: it is a genuine press.
		 *       Update last_press_ms = evt.timestamp_ms and fall through.
		 * ─────────────────────────────────────────────────────────────── */

		/* ── TODO 3 ────────────────────────────────────────────────────
		 * Advance the mode safely under a mutex.
		 *
		 * current_mode is shared with led_task.  Without a mutex, a read
		 * in led_task could interleave with your write here and observe a
		 * corrupt intermediate value (V2 Bug 3).
		 *
		 * Steps:
		 *   1. Lock:    k_mutex_lock(&mode_mutex, K_FOREVER)
		 *   2. Update:  current_mode = (led_mode_t)((current_mode + 1) % MODE_COUNT)
		 *   3. Capture: copy current_mode into local variable new_mode
		 *               (so you can printk after releasing the lock)
		 *   4. Unlock:  k_mutex_unlock(&mode_mutex)
		 *
		 * Keep the critical section short — only lines that touch
		 * current_mode belong inside the lock/unlock pair.
		 * ─────────────────────────────────────────────────────────────── */
		led_mode_t new_mode = MODE_OFF;   /* placeholder — replace with mutex-protected update */

		/*
		 * printk is SAFE here because we are in a thread, not an ISR.
		 * In V2, this same call is made from inside the ISR — wrong.
		 */
		printk("[Button] Mode -> %s\n", mode_names[new_mode]);
	}
}

/* ═══════════════════════════════════════════════════════════════════════════
 * LED THREAD
 *
 * Responsibilities:
 *   1. Read current_mode under mutex (short critical section).
 *   2. Produce one frame of the correct animation pattern.
 *   3. Sleep for LED_STEP_MS — button presses are handled independently.
 *
 * The LED thread does NOT own the mode variable — it only reads it.
 * The button thread is the sole writer.  This separation means LED animation
 * speed and button responsiveness are completely decoupled.  Contrast with
 * V1, where a single k_msleep() controls both at the same time.
 * ═══════════════════════════════════════════════════════════════════════════ */

static void led_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/*
	 * PINGPONG sequence indices into the LED array: 0→1→2→1→0→1→2→1→…
	 * Declared static so it lives in flash rather than on the thread stack.
	 */
	static const uint8_t pingpong_seq[] = { 0, 1, 2, 1 };

	uint32_t   step      = 0;
	led_mode_t prev_mode = MODE_COUNT;   /* impossible sentinel value        */

	while (1) {
		/* ── TODO 4 ────────────────────────────────────────────────────
		 * Read the shared mode safely.
		 *
		 * current_mode is written by button_task.  You must lock the
		 * mutex before reading it, copy the value into the local variable
		 * 'mode', then unlock.  Release the lock before the switch below
		 * — never hold a mutex longer than necessary.
		 *
		 * Steps:
		 *   1. Lock:    k_mutex_lock(&mode_mutex, K_FOREVER)
		 *   2. Copy:    mode = current_mode
		 *   3. Unlock:  k_mutex_unlock(&mode_mutex)
		 * ─────────────────────────────────────────────────────────────── */
		led_mode_t mode = MODE_OFF;   /* placeholder — replace with mutex-protected read */

		/* Reset the step counter when the mode changes so every pattern
		 * always starts from its beginning when first entered.
		 */
		if (mode != prev_mode) {
			step      = 0;
			prev_mode = mode;
		}

		/* ── Animate one frame ──────────────────────────────────────────
		 * Each case produces one step of its pattern.  The step counter
		 * is incremented at the bottom so all cases see the same value.
		 */
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
}

/* ═══════════════════════════════════════════════════════════════════════════
 * THREAD DEFINITIONS
 *
 * K_THREAD_DEFINE is a compile-time macro that creates the thread stack,
 * thread control block, and starts the thread automatically — before main()
 * is even called.
 *
 * Parameters: (name, stack_bytes, entry_fn, p1, p2, p3, priority, opts, delay)
 *
 * Priority: lower number = higher priority (runs first when both are ready).
 *   Button thread (4) > LED thread (5) — button events processed promptly.
 * ═══════════════════════════════════════════════════════════════════════════ */

K_THREAD_DEFINE(btn_tid, 1024, button_task, NULL, NULL, NULL, 4, 0, 0);
K_THREAD_DEFINE(led_tid, 1024, led_task,    NULL, NULL, NULL, 5, 0, 0);

/* ═══════════════════════════════════════════════════════════════════════════
 * main() — Hardware Initialization Only
 *
 * main()'s sole job is to initialize GPIO and register the ISR.  There is
 * no application loop here.  Once setup is complete, main() returns.
 *
 * The Zephyr kernel does NOT exit when main() returns.  The two threads
 * defined above (btn_tid, led_tid) continue running forever.
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(void)
{
	printk("╔════════════════════════════════════╗\n");
	printk("║  CSE 352/452 — Lab 7               ║\n");
	printk("║  V3: Student Implementation         ║\n");
	printk("║  Press BTN0 to cycle LED modes      ║\n");
	printk("╚════════════════════════════════════╝\n");

	/* ── Verify GPIO devices are ready ──────────────────────────────────── */
	if (!gpio_is_ready_dt(&led0) || !gpio_is_ready_dt(&led1) ||
	    !gpio_is_ready_dt(&led2) || !gpio_is_ready_dt(&btn)) {
		printk("ERROR: GPIO device not ready!\n");
		return -1;
	}

	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&btn, GPIO_INPUT);

	gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&btn_cb_data, button_isr, BIT(btn.pin));
	gpio_add_callback(btn.port, &btn_cb_data);

	printk("Hardware initialized. Threads running. main() returning.\n");
	return 0;
}
