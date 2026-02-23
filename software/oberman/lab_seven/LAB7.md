# CSE 352/452 — Lab 7: Event-Driven Embedded Design

## Overview

In this lab you will implement an event-driven LED mode controller on the
nRF52 DK with Berkeley Buckler Rev C daughter board.  The same application is
written three ways, each exposing a fundamental design lesson:

| File | Design | Problem demonstrated |
|------|--------|----------------------|
| `src/main_v1_polling.c` | Single polling loop | Timing is coupled — one sleep controls both button and LEDs |
| `src/main_v2_isr_bad.c` | GPIO interrupt, bad ISR | Bounce, blocking call in ISR, no mutex |
| `src/main.c` | **Your implementation** | Correct event-driven design |

The LEDs cycle through five modes each time the button is pressed:

```
OFF → FLASH → CHASE → PINGPONG → BINARY → OFF → …
```

---

## Learning Objectives

By the end of this lab you will be able to:

1. Explain why a single polling loop cannot independently control two
   time-sensitive tasks.
2. Identify at least three rules that must not be broken inside an ISR.
3. Implement a minimal ISR that hands work off to a thread via a message queue.
4. Implement timestamp-based debounce in a thread.
5. Protect a shared variable between two threads using a mutex.

---

## Hardware

- **nRF52 DK** (PCA10040) with the Buckler Rev C daughter board seated on the
  Arduino headers.
- **Button**: the small tactile button on the Buckler board (P0.28).
  *Not* the DK's own SW1–SW4 buttons.
- **LEDs**: three LEDs on the Buckler (Red = P0.25, Yellow = P0.24, Blue = P0.23).

Connect a USB cable to the **nRF52 DK** (not the Buckler).

---

## Build and Flash

The build system is already configured.  You do not need to touch any
`CMakeLists.txt`, `prj.conf`, or overlay file.

### Open the nRF Connect terminal

All `west` commands below must run in the **nRF Connect terminal**, which has
the SDK toolchain in its PATH.  Press `Ctrl+Shift+P` and search for
**nRF Connect: Create Terminal**.

> **Windows note**: do **not** include the `PS C:\...>` prompt when pasting.
> Run each command on its own line — `&&` chaining does not work in
> Windows PowerShell.

The `west` commands are identical on all platforms:

```
west build -b nrf52dk/nrf52832 -p always
west flash
```

The first command performs a full pristine build (required the first time or
after any build-system change).  The second flashes the compiled binary.

For subsequent code changes an incremental build is faster:

```
west build -b nrf52dk/nrf52832
west flash
```

### Serial output

Open a serial monitor at **115200 baud** to see `printk` output.

In VS Code: install the **Serial Monitor** extension (publisher: Microsoft),
then click the **Serial Monitor** tab at the bottom panel.

- **Windows**: select the `JLINK CDC UART Port (COMx)` port visible in
  Device Manager under "Ports (COM & LPT)".
- **Linux**: the port is typically `/dev/ttyACM0` or `/dev/ttyUSB0`.
  If you get a permissions error, run `sudo usermod -aG dialout $USER` and
  log out/in once.

Set baud rate to **115200** and click **Start Monitoring**.

---

## Part 1 — Observe V1: The Polling Trap

### Setup

In the nRF Connect terminal, copy the V1 file over `main.c`, then build and flash.

**Windows (PowerShell):**
```
Copy-Item src/main_v1_polling.c src/main.c
west build -b nrf52dk/nrf52832 -p always
west flash
```

**Linux / macOS:**
```
cp src/main_v1_polling.c src/main.c
west build -b nrf52dk/nrf52832 -p always
west flash
```

### Experiment

Open `src/main_v1_polling.c` and find the `#define SLEEP_TIME_MS` near the top.
Run the firmware three times with different values:

| `SLEEP_TIME_MS` | Button feel | LED animation |
|-----------------|-------------|---------------|
| `50`            |             |               |
| `200`           |             |               |
| `500`           |             |               |

Fill in your observations in the table above.

### Discussion questions

Answer these before moving on.  You will reference them in your report.

1. At 50 ms: the button feels responsive.  Why do the LEDs flicker
   unpleasantly?

2. At 500 ms: the LEDs look smooth.  Why is the button now hard to use?

3. Is there a value of `SLEEP_TIME_MS` that makes *both* the button and the
   LEDs feel good at the same time?  Why or why not?

4. Look at the single `k_msleep(SLEEP_TIME_MS)` call at the bottom of the
   loop.  What two completely different jobs does it control simultaneously?
   What would you need to do architecturally to let each job run at its own
   rate?

### Restore the skeleton before continuing

**Windows (PowerShell):**
```
Copy-Item src/main_skeleton.c src/main.c
```

**Linux / macOS:**
```
cp src/main_skeleton.c src/main.c
```

---

## Part 2 — Observe V2: The Bad ISR

### Setup

**Windows (PowerShell):**
```
Copy-Item src/main_v2_isr_bad.c src/main.c
west build -b nrf52dk/nrf52832 -p always
west flash
```

**Linux / macOS:**
```
cp src/main_v2_isr_bad.c src/main.c
west build -b nrf52dk/nrf52832 -p always
west flash
```

### Observe

Press the button several times and watch both the LEDs and the serial output.

### Discussion questions

Open `src/main_v2_isr_bad.c`.  Three deliberate bugs are marked with
`/* BUG 1 */`, `/* BUG 2 */`, and `/* BUG 3 */`.

For **each** bug, answer:

**Bug 1 — No debounce**

1. What do you observe on the hardware when you press the button once?
2. What is mechanical switch bounce?  Why does it cause multiple ISR
   firings from one human press?
3. Why can't you fix bounce by adding a `k_sleep()` inside the ISR?

**Bug 2 — `printk()` in ISR context**

4. What is ISR context?  How does it differ from thread context?
5. Why is calling `printk()` inside an ISR potentially dangerous?
   (Hint: think about what `printk()` must do to send characters over UART.)
6. Where should `printk()` be called instead?

**Bug 3 — Shared variable with no mutex**

7. Two execution contexts write to and read from `current_mode`.
   Which two contexts?
8. Why does `volatile` alone not make this safe?
9. What mechanism does V3 use to protect `current_mode`?

### Restore the skeleton before continuing

**Windows (PowerShell):**
```
Copy-Item src/main_skeleton.c src/main.c
```

**Linux / macOS:**
```
cp src/main_skeleton.c src/main.c
```

---

## Part 3 — Implement V3: The Right Way

Open `src/main.c`.  You will find **four TODO sections**.  Complete them in
order — each one builds on the previous.

The file compiles and flashes without any changes, but the button does nothing
until all four parts are implemented.

### Architecture

```
  Button press
      │
      ▼
  [button_isr]  ← ISR context: TINY — timestamp and enqueue only
      │
      │  k_msgq_put(&btn_queue, &evt, K_NO_WAIT)
      ▼
  [btn_queue]   ← message queue in kernel memory
      │
      │  k_msgq_get(&btn_queue, &evt, K_FOREVER)
      ▼
  [button_task] ← thread context: debounce, mutex, printk
      │
      │  k_mutex_lock / write current_mode / k_mutex_unlock
      ▼
  [current_mode] ← shared variable (protected by mode_mutex)
      │
      │  k_mutex_lock / read current_mode / k_mutex_unlock
      ▼
  [led_task]    ← thread context: animate LEDs independently
```

---

### TODO 1 — ISR body (`button_isr`)

**Where**: inside `button_isr()`, in the `/* TODO 1 */` block.

**What to write** (3 lines):

```c
struct button_event evt = { .timestamp_ms = k_uptime_get() };
k_msgq_put(&btn_queue, &evt, K_NO_WAIT);
```

**Why**:

- `k_uptime_get()` returns the number of milliseconds since boot.  Capturing
  it here records exactly *when* the button edge occurred.  The thread will
  use this timestamp for debounce.
- `k_msgq_put(..., K_NO_WAIT)` copies the event into the queue without
  blocking.  If the queue is full (eight events backed up), the event is
  silently dropped — that is always safer than blocking the ISR.
- Nothing else belongs here.  The ISR fires and returns in microseconds.

**Test**: After this step only, build and flash.  Press the button.  The mode
will advance, but you will likely see it jump 2–5 steps per press (bounce).
That is expected — debounce is TODO 2.

---

### TODO 2 — Debounce (`button_task`)

**Where**: inside `button_task()`, in the `/* TODO 2 */` block, immediately
after `k_msgq_get`.

**What to write** (2 lines):

```c
if ((evt.timestamp_ms - last_press_ms) < DEBOUNCE_MS) {
    continue;
}
last_press_ms = evt.timestamp_ms;
```

**Why**:

- Every bounce event has a timestamp very close to the real press.  By
  rejecting events that arrive within `DEBOUNCE_MS` (250 ms) of the last
  accepted one, we discard the bounces without any `k_sleep()`.
- `last_press_ms` is updated only when a press is accepted, which re-arms the
  window for the *next* press.
- `continue` jumps back to `k_msgq_get`, where the thread blocks again
  consuming no CPU.

**Test**: After this step, build and flash.  Each button press should advance
the mode exactly one step.  If you still see double-steps, your timestamp
comparison may be reversed.

---

### TODO 3 — Mutex write (`button_task`)

**Where**: inside `button_task()`, in the `/* TODO 3 */` block, replacing the
`led_mode_t new_mode = MODE_OFF;` placeholder.

**What to write**:

```c
led_mode_t new_mode;

k_mutex_lock(&mode_mutex, K_FOREVER);
current_mode = (led_mode_t)((current_mode + 1) % MODE_COUNT);
new_mode = current_mode;
k_mutex_unlock(&mode_mutex);
```

**Why**:

- `current_mode` is read by `led_task` at the same time `button_task` writes
  it.  On 32-bit ARM, a 32-bit aligned write happens to be atomic at the
  hardware level, so you may not see corruption in practice — but the C
  standard does not guarantee this, and the mutex makes the intent explicit and
  portable.
- Keep the critical section short: only the lines that touch `current_mode`
  are inside the lock.  The `printk()` call below uses `new_mode` (a local
  copy) so it runs outside the lock, which is correct.
- `K_FOREVER` means "wait as long as needed to acquire the lock."  The LED
  thread holds the lock for only three lines, so the wait is always negligible.

**Test**: The `[Button] Mode -> ...` serial output should now show the correct
mode name (not always `OFF`).

---

### TODO 4 — Mutex read (`led_task`)

**Where**: inside `led_task()`, in the `/* TODO 4 */` block, replacing the
`led_mode_t mode = MODE_OFF;` placeholder.

**What to write**:

```c
led_mode_t mode;

k_mutex_lock(&mode_mutex, K_FOREVER);
mode = current_mode;
k_mutex_unlock(&mode_mutex);
```

**Why**:

- The LED thread only reads `current_mode`; `button_task` is the sole writer.
  Even so, a read that races with a write can return an intermediate value on
  some architectures.  The mutex prevents that.
- Critically, the lock is released **before** the animation switch below.
  Never do GPIO operations or `k_msleep()` while holding a mutex.
- Once `mode` is in a local variable, the thread works entirely from the local
  copy for the rest of the frame.  This is the correct pattern for a
  reader/writer relationship.

---

### Verification checklist

Build and flash your completed implementation.  Verify each item:

- [ ] Serial output shows the startup banner when the board resets.
- [ ] LEDs start in MODE_OFF (all dark).
- [ ] Each button press advances the mode exactly one step.
- [ ] FLASH: all three LEDs blink together at a visible rate.
- [ ] CHASE: one LED chases Red → Yellow → Blue → Red → …
- [ ] PINGPONG: one LED bounces Red → Yellow → Blue → Yellow → Red → …
- [ ] BINARY: three LEDs count 0–7 in binary, repeating.
- [ ] The fifth press returns to MODE_OFF.
- [ ] Pressing the button rapidly (holding it down) does not skip modes.
- [ ] LED animation does not pause or stutter when the button is pressed.

---

## Submission

Include in your report:

1. Your completed `src/main.c`.
2. The observation table from Part 1 (three SLEEP_TIME_MS values).
3. Written answers to all discussion questions in Parts 1 and 2.
4. A paragraph explaining in your own words why the V3 architecture solves
   the problems you observed in V1 and V2.
