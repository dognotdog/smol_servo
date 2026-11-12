# Servo loop design (RP2350 firmware)

Date: 2026-11-09. Scope: `firmware_rpico/`, board `smol-servo-rp-nodrv` with `smol-servo-nomcu-tmc6200`.

This note collects how the servo loop works now, and how it should work. Sections 1 to 5 describe the code as it is. Sections 6 to 9 are a proposal. Section 10 lists the decisions that the owner must make before the loop code.

Related notes:

- `firmware_rpico/doc/rpico-servo-loop.pdf` (owner's diagram, OmniGraffle): PWM, ADC, IRQ and loop timing, peripheral and DMA assignment. Section 11 lists where it and the code differ.
- `notes/motor-driver-model-analysis.md`: motor model, estimator and current-sense defects.
- `notes/usb-pd-reboot-loop.md`: PD startup order and the reason for the early PD sink start.

## 1. Status

| Part | Status | Location |
|---|---|---|
| Peripheral timing (PWM, ADC, MAG, loop IRQ) | Implemented and tuned with a scope | `smol_servo_pwm.c`, `smol_servo_adc.c`, `smol_servo_mag.c`, `smol_servo_parameters.h` |
| Loop IRQ and sensor reads | Implemented | `smol_servo_loop.c` |
| PWM output from the loop | Implemented. `smol_pwm_prepare()` (loop end) and `smol_pwm_fill()` (next loop start) write duty and enable per half bridge and PWM period. All half bridges are off in the states that do not drive the bridge. | `smol_servo_pwm.c` |
| Current conversion (ADC volts to amperes) | Draft, does not compile, not in the build | `smol_servo_isense.c` |
| Estimator (model, filter) | Draft, not in the build | `smol_servo_model.c` |
| Controller | Not started | — |
| Loop states | Implemented and tested at 20 V. See section 7.2. | `smol_servo_loop.h`, `smol_servo_loop.c` |
| Current sense zero calibration | Implemented and tested at 20 V. It is the first state after `smol_servo_loop_start()`. See section 7.1. | `smol_servo_loop.c` |
| Motor identification | Not implemented. The state `motor_id` is a placeholder. See section 7.2. | `smol_servo_loop.c` |
| Setpoint input | Not started | — |
| Power stage enable and fault handling | Implemented, not connected to the loop | `smol_servo_drv.c` |
| Loop start | `smol_servo_loop_start()` is commented out in `_core1_init`. The console command `s` calls it. | `main.c`, `smol_console.c` |

The loop IRQ handler reads all sensors. At the comment "This is our core servo loop" it has no code.

## 2. Execution context

- **Core 1** runs the servo loop and all its interrupts: bridge PWM wrap, ADC FIFO and the loop IRQ. `smol_irq_init()` enables them on core 1.
- After the init, core 1 has no access to the XIP flash (`accessctrl_hw->xip_main`). All core 1 code and data must be in RAM (`__not_in_flash_func`, `__not_in_flash`). This prevents XIP stalls and IRQ jitter. A flash access from core 1 causes an exception (`_exception_dummy_handler`).
- **Core 0** runs the USB PD sink, the console, the log output, and all TMC6200 SPI access (`smol_drv_*`).
- Core 1 log lines go through a queue. Core 0 prints them in `picobug_process_core1_queue()`.

Interrupt priorities (lower value = higher priority), `smol_servo_parameters.h`:

| IRQ | Priority | Function |
|---|---|---|
| ADC FIFO (4 samples) | 0x10 | `smol_adc_fifo_threshold_irq_handler` |
| Bridge PWM wrap | 0x20 | `smol_bridge_pwm_wrap_irq_handler` |
| Servo loop (user IRQ, set pending by software) | 0x80 | `smol_servo_loop_irq_handler` |

The loop IRQ has the lowest priority. The ADC and PWM interrupts can interrupt it.

## 3. Timing

System clock 150 MHz.

| Signal | Frequency | Period | Clocks | Define |
|---|---|---|---|---|
| Servo loop | 20 kHz | 50 µs | 7500 | `SMOL_SERVO_LOOP_*` |
| Bridge PWM (up/down) | 60 kHz | 16.67 µs | 2500 (TOP + 1 = 1250) | `SMOL_SERVO_BRIDGE_PWM_*` |
| ADC sample | 240 kHz | 4.17 µs | 625 | `SMOL_SERVO_ADC_*` |
| AS5047D SPI transaction | 80 kHz | 12.5 µs | 1875 | `SMOL_SERVO_MAG_*` |

Thus one loop period has 3 bridge PWM periods, 12 ADC samples and 4 encoder transactions.

PWM slices (`boards/smol_servo_rpico.h`):

| Slice | Name | Function |
|---|---|---|
| 3 | PWMR | Brake resistor, 60 kHz up/down |
| 4, 5, 8 | PWMA, PWMB, PWMC | Bridge phases, 60 kHz up/down. A = PWM (H input), B = EN (L input). |
| 6 | PWM_ADC | ADC trigger, 240 kHz. Its counter starts 150 clocks early. |
| 7 | PWM_SERVO_LOOP | 20 kHz |
| 9 | PWM_DOUBLETIME | 120 kHz, not phase-correct. Not used: no pin, DMA or IRQ. Probably obsolete, kept for now (owner, 2026-11-09). |
| 10 | PWM_MAG | AS5047D transaction trigger, 80 kHz. Its counter starts with an offset that aligns its phase with the bridge PWM. |

DMA channels (the code claims them with `dma_claim_unused_channel()`, so the numbers can change):

| Function | Trigger | Transfer |
|---|---|---|
| ADC trigger | PWM_ADC wrap | ADC.CS values from a ring buffer of 4 blocks to `adc_hw->cs`, endless |
| Loop timestamp | PWMA wrap | `timer1_hw->timerawl` to `_timer1_sample_dma_buf` |
| AS5047D TX, RX | PWM_MAG wrap, SPI | Command and reply buffers |

The ADC results do not use DMA. The ADC FIFO IRQ reads them. The bridge CC values do not use DMA. The PWM wrap IRQ writes them.

Timing diagram (from `smol_servo_parameters.h`):

```
LOOP ...  t=0                                             t=1...
20k       |                                               |
PWM  ...  0               1               2               0  ...
60k       |               |               |               |
MAG  ...  0           1           2           3           0  ...
80k       |           |           |           |           |
ADC  ... 0.0 0.1 0.2 0.3 1.0 1.1 1.2 1.3 2.0 2.1 2.2 2.3 0.0 ...
240k     |   |   |   | : |   |   |   | : |   |   |   | : |
ADC-IRQ                x               x               x
```

### 3.1 Current samples

- In bridge PWM period k (k = 0, 1, 2), the ADC samples the current of phase k two times. The samples are at counter 0 and at counter TOP. The two other ADC samples in that period read auxiliary channels.
- ADC order in block k: `I_k(0)`, `aux(2k+3)`, `I_k(TOP)`, `aux(2k+4)`. The auxiliary channels are 3 VBUS, 4 VDD, 5 to 7 ADCA to ADCC, 8 chip temperature.
- The ADC is a point sampler: it holds the input approx. 0.4 µs after the trigger (CS write with START_ONCE). The trigger is 60 clocks (0.4 µs) early (`SMOL_SERVO_ADC_ADVANCE_COUNT`), so that the hold is at the PWM counter 0 or TOP. The conversion then takes 2 µs.
- The PWM output is high while COUNT < CC (owner's diagram). Thus the sample at counter 0 is in the middle of the high-side on time. The sample at TOP is in the middle of the low-side on time. The two samples can have different offsets (`n17-servo-design.tex:517`).
- The ADC FIFO IRQ comes after each block of 4 samples, about half an ADC period before the next bridge PWM wrap. The ADC data is thus ready before the PWM IRQ and the loop start. The owner's diagram marks this as a latency reduction.
- Thus each phase current has 2 samples per loop, and the 3 phases are sampled at different times. They are not a simultaneous set.
- The CS ring buffer has 4 blocks, and the pattern has 3 blocks. The DMA runs a ring slot again 4 blocks later. Thus the FIFO IRQ refills the slot that just ran with pattern `(block + 4) % 3`, and stores the samples at the index of the block that ran. Before 2026-11-09, the refill used the pattern that had just run. Phase 2 then ran two times (blocks 3 and 4), and after that the phase order was one bridge PWM period late at each loop, permanently.
- Measured sample ages at the loop IRQ after the correction (clocks, counter 0 / TOP): phase 0 7448 / 6198, phase 1 4946 / 3696, phase 2 2446 / 1196. Phase 2 is the newest, as the startup alignment intends. Before the correction, phase 1 was the newest and phase 2 was one loop (7461 clocks) old.
- The TMC6200 amplifier output has a spike of 2 µs to 4 µs after each switching edge (datasheet 6.1). `smol_servo_isense.c` selects the sample that has more than `ISENSE_SETTLING_CYCLES` (600 clocks, 4 µs) of margin to an edge.
- `smol_adc_get_isense_values()` returns the 3 × 2 samples in volts, with a timestamp (timer 1, sysclk) for each sample.

### 3.2 Encoder samples

- 4 transactions per loop: `DIAAGC`, `ANGLEUNC`, `MAG`, `ANGLEUNC`. The AS5047D returns the data of a command in the next transaction.
- `smol_mag_get_reading_values()` returns 2 `ANGLEUNC` values per loop.
- `smol_mag_get_reading_ages_ns()` gives their ages: 150.49 µs and 125.49 µs. Each age is the read time (−50.49 µs and −25.49 µs) plus the AS5047D processing delay (100 µs typ., 90 µs to 110 µs).

### 3.3 Loop trigger and latency

- The PWM wrap IRQ writes the CC values for the period after the next one. After the period that samples phase 2, it sets the loop IRQ pending (`smol_servo_signal_loop()`).
- The loop then has the current samples of all 3 phases from the last 3 PWM periods.
- Latency from sensor sample to PWM output: 5 bridge PWM periods (83 µs). See the comment in `smol_servo_loop_irq_handler`.
- **Measured loop timing** (2026-11-10, motor id pulse test at 625 cc; t = 0 at the bridge PWM wrap (counter 0) of the loop signal; figures `scope/loop-fill-timing.png`, `scope/loop-end-sweep.png`, `scope/adc-irq-end-sweep.png`):

  | Event | Time |
  |---|---|
  | PWM wrap IRQ (sets the loop IRQ pending) | +0.17 µs, 0.8–1.0 µs long, again at 16.84 µs and 33.5 µs |
  | Loop entry | 1.15 µs after the PWM IRQ entry |
  | `smol_pwm_fill()` done (only copies 9 words) | approx. 0.7 µs after the entry |
  | ADC read (`smol_adc_get_isense_values()`) | just after the fill |
  | Loop end, MOTOR_ID, no test delay | 13.3 µs mean, 13.5 µs max (some segments 9.5 µs) |
  | ADC FIFO IRQ (end of the 4th conversion of a block) | start 14.4 µs, end 16.3 µs (16.2–16.6 µs); again at +16.67 µs |
  | ADC triggers / holds | trigger 0.4 µs before each hold. Holds: A0 0, A1 8.3, B0 16.7, B1 25, C0 33.3, C1 41.7 µs |

  The ADC IRQ has priority over the PWM IRQ and the loop. Thus no loop end is possible from approx. 14.1 to 17.6 µs and from 30.8 to 34.3 µs.
- **The loop converts the duty to cc at its end.** `smol_pwm_prepare()` converts `_bridge_out` to the CC buffer words (`smol_bridge_cc_t`, approx. 2 µs for 9 float conversions). `smol_pwm_fill()` at the start of the next loop only copies them, and `_cc_hist_push()` takes the cc for the current sense table from them. The margin from the fill to the PWM IRQ that loads k 0 is more than 13 µs. Thus the fill is not time critical.
- **Rule 1: the ADC read must come before the ADC FIFO IRQ of the next phase A block (approx. 14.1 µs).** A later read gets the new A block in some loops. A test with a 10 µs delay before the read gave A1 a sigma of up to 226 mA and a false d of +140 / −50 mA. With the read at the loop start, this cannot occur. The read is not protected (the IRQ can write the buffer during the read). Only the timing prevents the race.
- **Rule 2: the loop end must not meet the arrival of the ADC FIFO IRQ.** Sweep of a test delay at the loop end (`loop_end_delay`, 0..3600 clocks in steps of 30, pulse test B->C, 1024 cycles per point):
  - Loop end 13.9 µs (mean, max 14.1 µs): B0 sigma 75 mA (normally 18 mA), d −37 mA. 0.2 µs earlier or later: normal. The same for C0 at a loop end of 30.4–30.7 µs (C0 sigma 73 mA, d +22 mA). The window is approx. 0.3 µs wide. Then the IRQ tail-chains after the loop instead of a preemption.
  - A test build with out-of-line calls in the fill ended its loop just after the ADC IRQ arrival. B0 then had a sigma of 80–130 mA and a false d. This was the first observation.
  - The end of the ADC IRQ is not the cause: a test delay at the end of the ADC IRQ (`adc_irq_delay`, 0..300 clocks) moved the IRQ end from 16.3 to 18.5 µs, through the B0 / C0 acquisition window. B0 and C0 did not change (sigma 16–18 mA, d ±2 mA).
  - The mechanism is not known. A next test: put the ADC IRQ debug GPIO (12) on a scope line and compare a good and a bad loop end.
  - Without a test delay, the loop end (13.5 µs max) is only approx. 0.4 µs before this window. A longer loop (for example the control state) must end well before 13.9 µs or after the PWM IRQ (17.6 µs).
- **Rule 3: the loop end should not be at an ADC trigger.** In the sweep, B1 changed at a loop end of 24.5 µs (sigma 55 mA, d −3.5 instead of +15 mA), and C1 at 41.0–41.3 µs (sigma 54 mA). At all other loop end times, no sample changed.
- **The noise of a sample also depends on the code that core 1 executes in the loop.** With the loop still running at the ADC IRQ arrival, B0 sigma was 16.5–26.7 mA in different builds (±0.5 mA in one build). A probable cause: the IRQ entry latency depends on the instruction that it interrupts (rule 2). Not confirmed.
- **Compare pulse tests only at the same VBUS.** The current steps scale with VBUS. Between two runs, the USB PD contract changed from 15 V to 20 V (probably a run-time `pd_req` that stayed over the SWD resets until a new negotiation), and B1 d changed by ×1.39. The automatic sequence (`id_auto`) divides by the measured VBUS. The single `id_pulse` prints only d.
- At the end of the handler, `assert_fast` checks that the loop IRQ is not pending again. If it is pending, the loop took more than one period.

### 3.4 Time budget

- 7500 clocks per loop at 150 MHz.
- The ADC IRQ (every 4 samples, 60 kHz) and the PWM IRQ (60 kHz) use part of this time. Measured 2026-11-10: the ADC IRQ takes approx. 1.9 µs (from its timer read to its end, 1.85 µs mean, 2.17 µs max) and the PWM IRQ 0.8–1.0 µs, each 3 times per loop. Together approx. 8.5 µs of the 50 µs. The ADC IRQ is long for its work (4 FIFO reads, some buffer writes). The cause is not known. GPIO 8, 9, 12 and 13 can show the IRQ times on a scope (`DEBUG_*_WITH_GPIO`). The console commands `loop_end_delay` and `adc_irq_delay` print the measured times.
- The Cortex-M33 has a single-precision FPU. Double precision is software emulation and must not be used in the loop.

## 4. Loop interface (now)

Inputs at the start of `smol_servo_loop_irq_handler()`:

| Input | Type | Source |
|---|---|---|
| Phase current samples | `float[3][2]` volts, `uint32_t[3][2]` timestamps | `smol_adc_get_isense_values()` |
| Rotor angle | 2 × raw `ANGLEUNC`, 2 ages in ns | `smol_mag_get_reading_values()`, `smol_mag_get_reading_ages_ns()` |
| Loop timestamp | low 32 bit of timer 1, sampled by DMA at the PWMA wrap | `_timer1_sample_dma_buf[0]` |
| Bus voltage, temperature | volts, °C | `smol_adc_vbus()`, `smol_chip_temperature_celsius()` |

Output:

- `smol_pwm_prepare(const smol_bridge_out_t*, smol_bridge_cc_t*)` converts the duty and enable of each half bridge for the next 3 PWM periods to CC buffer words. The loop calls it at its end. `smol_pwm_fill(const smol_bridge_cc_t*, starting_phase)` copies the words into `_ram.pwm_cc_loop_bufs`. The loop calls it at its start with `starting_phase = 2`. One 32-bit store writes both CC values of an entry.
- `smol_pwm_all_off()` sets all bridge CC buffers and registers to 0 at once (fault path).
- `smol_servo_loop.h` defines `smol_bridge_out_t`, see section 7.2.
- Each bridge slice has 2 CC values: A is the PWM (TMC6200 H input) and B is the enable (L input, `singleline = 1`). An EN value of 0 switches the half bridge off (output floating).

## 5. Power stage interface (now)

- `smol_drv_enable()` and `smol_drv_disable()` run on core 0. The enable needs a PD contract of 9 V or more. It warns below 10 V (TMC6200 datasheet 11.1).
- `smol_drv_run()` in the core 0 main loop disables the driver on a low contract voltage or on a FAULT.
- The loop on core 1 has no direct connection to this. Section 7 proposes one.

## 6. Proposed data flow per loop

```
ADC samples (3x2) ──> current conversion ──┐
                      (zero, gain, valid)  │
AS5047D angles (2) ──> angle unwrap ───────┼──> estimator ──> state estimate ──> controller ──> phase voltages ──> PWM duty ──> smol_pwm_fill()
VBUS ──────────────────────────────────────┘    (predict to                      (setpoint)     (with U_bus,      (3 phases x
                                                 each sample time)                               dead time)        3 periods)
```

1. **Current conversion.** `i = (v − v_zero) / (G · R_sense)`. G is from `ISENSE_AMPLIFICATION_CODE` (20x). R_sense is 5 mΩ. The zero comes from the calibration (section 7, step 4). Use the sample with enough margin to the edges, as `smol_servo_isense.c` does.
2. **Angle.** Convert the 14-bit `ANGLEUNC` to radians and unwrap it to a multi-turn position. Use the age of each sample in the estimator.
3. **Estimator.** Predict the state to the time of each measurement, then update it. The current samples come at 6 different times per loop. `notes/motor-driver-model-analysis.md` section 5.1 to 5.4 describes the model and the defects to correct first.
4. **Controller.** It calculates the phase voltages from the state estimate and the setpoint. Section 10 lists the open decision about its type.
5. **PWM.** Duty = U_phase / U_bus, limited to the range that keeps the valid current sample (section 3.1). Predict the rotor angle to the middle of the PWM period that applies the duty (latency 83 µs, section 3.3). Correct for the dead time (`smol_drv_bbm_ns()`, the config key `bbmclks` times 42 ns).

## 7. Proposed modes and startup sequence

| Step | Mode | Core | Action | Condition to continue |
|---|---|---|---|---|
| 1 | Boot | 0 | `smol_drv_init()` with DRV_EN low, then the PD sink startup | PD startup complete or timeout |
| 2 | Peripheral start | 1 | ADC, MAG and PWM start with all EN = 0 (bridge off). Loop IRQ runs, estimator off. | AS5047D detected, ADC IRQ period in range |
| 3 | Wait for power | 0 | Wait for a PD contract | `smol_usb_pd_contract_mv()` ≥ 9000 |
| 4 | Zero calibration | 1 and 0 | With DRV_EN low, average the current samples of each channel, at counter 0 and at TOP separately (section 7.1) | Zero inside 50 mV × gain around `VSENSE_NOMINAL_ZERO_V`, sigma ≤ 20 mV |
| 5 | Enable | 0 | `smol_drv_enable()` | Returns 0 |
| 6 | Motor discovery | 1 | R and L per phase, rotor alignment, encoder offset, pole pairs (design doc section "Motor Discovery Implementation") | Parameters inside a valid range |
| 7 | Closed loop | 1 | Estimator and controller run | — |
| F | Fault | 0 and 1 | EN = 0 at once (core 1), then `smol_drv_disable()` (core 0), log the cause | Command from the console |

### 7.1 Current sense zero calibration (implemented)

- `smol_servo_loop_start()` (core 0) refuses the start while the TMC6200 is enabled. It starts MAG, PWM and ADC, then requests the state `zero_cal`. A second call does only a new calibration.
- In the state `zero_cal`, core 1 skips 200 loops (10 ms), then adds 4096 loops (205 ms) of the 6 samples. It adds the difference to `VSENSE_NOMINAL_ZERO_V`, so that the float sums keep their resolution.
- After the state, core 0 (`_zero_cal_evaluate()`) calculates the mean and the standard deviation, checks the limits and logs the result. Then it gives the zeros and the scale `1 / (gain × ISENSE_R)` to the loop. The overcurrent check then converts the samples to amperes. Positive current flows into the motor terminal (TMC6200 datasheet 6).
- `smol_drv_enable()` refuses the enable while the calibration is busy.
- `smol_drv_set_isense_gain()` makes the zeros invalid. If the loop runs, a new calibration starts.
- `smol_servo_loop_isense_zero()` gives the zeros to the current conversion. It returns false when they are not valid.

Measured on the board (2026-11-09, 20 V, DRV_EN low):

| Phase | Zero at gain 5 | Zero at gain 10 | Zero at gain 20 | Input offset | Sigma at gain 20 |
|---|---|---|---|---|---|
| 0 | 1.711 V | 1.759 V | 1.849 V | 9.1 mV | 8 mV |
| 1 | 1.758 V | 1.847 V | 2.027 V | 18.1 mV | 2.5 mV |
| 2 | 1.746 V | 1.824 V | 1.979 V | 15.6 mV | 3.1 mV |

- The input offset is the same at all 3 gains. Thus the zero is `VSENSE_NOMINAL_ZERO_V + gain × VINOFS`, as the datasheet (sections 6 and 6.2.1) says. VOFS = 5VOUT/3 is correct.
- The counter 0 and TOP zeros differ by less than 1 mV.
- Phase 1 at gain 20 has 18 mV × 20 = 0.36 V of offset. The positive output range is then 0.36 V smaller: approx. 12.7 A in one direction and 20 A in the other (5 mΩ, 3.3 V ADC range).
- Phase 0 has approx. 3 times more noise than phases 1 and 2. The cause is not known.
- Not done: thermal drift compensation (datasheet 6.2.2).

### 7.2 Loop states (implemented)

The loop has a fixed set of states (`smol_loop_state_t` in `smol_servo_loop.h`). Core 0 requests a state with `smol_servo_loop_request_state()`. Core 1 executes the state in each loop IRQ, in one `switch` in `smol_servo_loop_irq_handler()`. A bounded state advances itself: `zero_cal` goes to `zero_cal_done`, and an overcurrent goes to `fault`. Code on both cores can check the state with `smol_servo_loop_state()` (in RAM). A state has no entry function: core 0 prepares the data of a state before it requests the state.

| State | Bridge | Set by | Precondition (core 0) | Next |
|---|---|---|---|---|
| `off` | off | core 0 | — | — |
| `zero_cal` | off | core 0 | TMC6200 disabled | `zero_cal_done` (core 1) |
| `zero_cal_done` | off | core 1 | — | `off` (core 0, after the evaluation) |
| `motor_id` | (not implemented, off) | core 0 | TMC6200 enabled, zero calibration valid | — |
| `control` | (not implemented, off) | core 0 | TMC6200 enabled, zero calibration valid | — |
| `pwm_test` | test pattern | core 0 | — | — |
| `fault` | off | core 1 | — | `off` (core 0) |

- **Variables.** Core 0 writes `_req_state`. Core 1 writes `_state`. Core 1 takes `_req_state` into `_state` when `_req_state` changes (it compares with `_req_taken`, which only core 1 uses). Thus a request of the same state again has no effect. Core 0 requests `off` after each `zero_cal_done` and `fault`. A new `zero_cal` request always goes through `off` first (see below), so it restarts a running calibration.
- **Core 0 jobs** (`_core0_run()`, nothing to do while `off` is requested): in `zero_cal_done`, evaluate the sums (`_zero_cal_evaluate()`, which needs `sqrtf` and `printf`) and request `off`. In `fault`, disable the TMC6200, log the fault and request `off`. In `motor_id` or `control`, request `off` if the TMC6200 is disabled (for example by `smol_drv_run()` after a FAULT pin).
- **Zero calibration.** Core 0 requests `off`, waits 2 loops (100 µs), clears the sums and the loop count `_zero_cal_loops`, then requests `zero_cal`. Core 1 adds up the samples and increments the count. At `ZERO_CAL_LOOPS`, core 1 sets `zero_cal_done`.
- **Overcurrent.** In `motor_id`, `control` and `pwm_test`, if a phase current is above `SMOL_ISENSE_FAULT_A` (8 A), core 1 calls `smol_pwm_all_off()` and sets `fault`. The check uses the smaller magnitude of the 2 samples of a phase, because a switching spike can affect one sample. The check does not run in `zero_cal`: the bridge is off there, and core 0 writes the calibration in `zero_cal_done`. Thus core 1 never reads the calibration while core 0 writes it. In `fault`, core 0 accepts only a request of `off`.
- **Barriers.** `__dmb()` (a `dmb` instruction and a compiler barrier) is necessary where non-volatile data goes from one core to the other with a volatile variable: the zero calibration sums before `zero_cal_done` (core 1) and after it (core 0), the cleared sums before the `zero_cal` request, and the calibration before `_cal_valid`. The fault values are volatile, so they need no barrier.
- **Calibration.** Core 0 writes the zeros and the scale `1 / (gain × ISENSE_R)` in `zero_cal_done`. No bridge state runs at that time. A gain change makes the zeros invalid and requests `zero_cal`.
- **Inputs and outputs.** The loop IRQ reads its inputs into local variables: the current sense voltages with their times (`isense_values`, `isense_times_clocks`), the loop time (`now_clocks`), and the raw ANGLEUNC words with their ages (`mag_reading_values`, `mag_reading_ages_ns`). The states use them directly. `smol_bridge_out_t`: duty (high-side on time) and enable for each half bridge and each of the 3 PWM periods. The loop end converts the outputs (`smol_pwm_prepare()`), and the next loop writes them with `smol_pwm_fill()`.
- **To add a state:** add it to the enum, to `_state_names[]` and to the `switch`, and add its preconditions to `smol_servo_loop_request_state()`. A bounded state sets its own DONE state in the `switch`, and `_core0_run()` acts on the DONE state.

Rules for core 1 code (core 1 has no XIP access):

- Mark functions `__not_in_flash_func` and const tables `__not_in_flash`. Also mark static and header helpers with `__not_in_flash_func` (`static inline` in a header), so that an out-of-line copy goes to RAM. Do not use `__force_inline` for this. `tools/check_core1_ram.py` checks the build.
- Do not call `printf`, libm, `memset` or `memcpy`. `smol_sin()` is in RAM.
- After a change, run `firmware_rpico/tools/check_core1_ram.py firmware_rpico/build/smol_servo_rpico.elf`. It follows all direct calls from the core 1 IRQ handlers in the disassembly and reports calls to flash and flash literals.

Console commands: `s` (start, zero calibration), `stop` (request `off`), `state` (print), `state <name>` (request a state).

Test (2026-11-10, 20 V):
- Before `s`, `state pwm_test` was refused. `s` did the zero calibration (approx. 250 ms): `zero_cal`, `zero_cal_done`, then `off`.
- `state zero_cal_done` and `state fault` were refused.
- A second `s` 50 ms after the first one restarted the calibration. Only one result came, 254 ms after the second start.
- `cfg_set isense_gain 10` started a new zero calibration.
- `pwm_test` ran with the TMC6200 disabled and enabled (no false fault), and stopped with `stop`.
- With `drv_en`: `motor_id`, `pwm_test`, then `control`. `drv_dis` in `control` requested `off`.
- Not tested: a real overcurrent.

Communication between the cores:

- Core 1 owns the mode. Core 0 sends requests (enable, disable, setpoint, mode change) through a queue or the SIO FIFO.
- Core 0 sends the PD contract voltage and the driver state to core 1, for example as a `volatile` value that only one core writes.
- Core 1 can set all EN to 0 by itself, without core 0. This is the fastest stop. Core 0 then sets DRV_EN low.

## 8. Proposed supervision

- **Loop overrun.** Count overruns, do not only assert. After N overruns, go to the fault mode.
- **ADC IRQ period.** `smol_servo_adc.c` has `_adc_irq_period_clocks_min/max`. Use them to detect a loss of synchronization.
- **Encoder.** Check `DIAAGC` and the `MAG` value in each loop. Go to the fault mode if the AS5047D reports an error.
- **Current limit.** Go to the fault mode if a phase current is above a limit for more than a set time.
- **VBUS.** Compare the measured VBUS with the PD contract voltage.
- **Driver.** `smol_drv_run()` already handles the FAULT pin and the contract voltage.

## 9. Gaps in the current code

1. (Done) `smol_pwm_fill()` has a body, see section 4.
2. `smol_servo_isense.c` and `smol_servo_model.c` do not compile and are not in the build.
3. `smol_servo_loop_start()` is commented out in `_core1_init`. The console command `s` starts the peripherals from core 0. The interrupts still run on core 1, but the start is outside the mode control of core 1.
4. `smol_adc_vbus()` reads 0.000 because the ADC does not run.
5. (Done) The loop states (section 7.2) connect the loop with the driver enable. The closed loop (`control`) has no implementation yet.
6. The documentation in `doc/n17-servo-design/n17-servo-design.tex` describes the STM32G4 and DRV8323 hardware (TIM2, 50 kHz, low-side shunts). `doc/smol-servo-modeling/smol-servo-modeling.tex` has only an introduction.
7. `as5047d_detect()` fails in approx. 1 of 7 boots: the first SPI frame after the boot returns 0xC000, and ERRFL reads 0x4004 (PARERR). The detection requires a clean NOP answer. When it fails, the MAG DMA does not start.

## 10. Decisions for the owner

1. **Controller type.** The README lists LQG. The alternative is a cascade of PI controllers (current, velocity, position), as in Moteus and ODrive (design doc, survey section). LQG uses the full state estimate and the model. A cascade is easier to tune without a good model.
2. **Control frame.** Field-oriented control in the DQ frame (current control on i_d and i_q), or direct control of the phase voltages from the state-space model.
3. **Setpoint input.** The options on this board:
   - Console commands (for development).
   - ENCA, ENCB and ENCI (GPIO 3 to 5): an external A/B encoder input, or step/direction.
   - EXT I2C (GPIO 14, 15) or USB.
   - SYNC (GPIO 16) for multi-axis synchronization.
4. **Motor types first.** 2-phase stepper on the 3-phase bridge (NEMA17, 50 pole pairs), 3-phase BLDC, or both. The model and the discovery steps depend on this.
5. **Current sense path.** The analog path from the TMC6200 outputs to the ADC pins (divider ratio, zero). This is open question 1 in `notes/motor-driver-model-analysis.md`.
6. **Dead time.** The config keys `bbmclks` (0..15, 42 ns each) and `drvstrength` (0..3) set DRV_CONF (`smol_drv_set_switching()`, only with the TMC6200 disabled). Use `cfg_set` to tune them on the scope without a new flash. The defaults stay at the reset values: BBMCLKS 4 (168 ns) and DRVSTRENGTH 2.
   - Scope measurement still to do: BMx and the gate signals, with the load current pulling the output up and down, at DRVSTRENGTH 2, 1 and 0. Then set the shortest BBMCLKS that covers the edges, plus 30 % (TMC6200 datasheet, "DRV_CONF Parameters" after Figure 3.5).
   - The datasheet recommends DRVSTRENGTH 0 or 1 for a Miller charge below 10 nC. The NTTFS5C658NL datasheet (the non-automotive version of the NVTFS5C658NL) gives Qgd 2.4 nC. Confirm it in the NVTFS5C658NL datasheet.
   - The model must include the dead time. Each switching half bridge has a voltage error of approx. `sign(i) · t_BBM · f_PWM · VS` (1 % of VS at 168 ns and 60 kHz: 0.2 V at 20 V). Across a coil between 2 switching half bridges, the error can be 2 times this value. The motor identification must compensate it, or use 2 current levels for R, so that a constant error cancels.

## 11. Differences between the owner's diagram and the code

Checked on 2026-11-09 against `firmware_rpico/doc/rpico-servo-loop.pdf`. The diagram and the code agree on these items:

- the PWM up/down mode,
- the ADC order (0, 3, 0, 4, 1, 5, 1, 6, 2, 7, 2, 8),
- the 150-count ADC advance and the shorter first ADC block,
- the ADC FIFO IRQ, the PWM feeder IRQ and the 20 kHz loop IRQ.

These items differ:

1. **IRQ priorities.** The diagram gives "0: DMA0, PWM0" and "1: SPAREIRQ". The code uses ADC FIFO 0x10, bridge PWM wrap 0x20 and loop 0x80. The DMA IRQ exists only with `DEBUG_DMA_ADC`.
2. **PWM feeder IRQ name.** The diagram box says "IRQ PWM4.TOP". The code uses the wrap IRQ of slice 4 (`PWM_IRQ_WRAP_0`). The timeline in the diagram shows the IRQ at counter 0. Check which point the RP2350 wrap IRQ uses in phase-correct mode, and make the box agree.
3. **PWM slices.** The diagram lists slices 0 to 8. The code also uses slice 9 (PWM_DOUBLETIME) and slice 10 (PWM_MAG, 80 kHz).
4. **DMA numbers.** The diagram uses fixed numbers, and "DMA1" and "DMA2" have two functions each. The code claims free channels at run time.

