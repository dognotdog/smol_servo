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
| PWM output from the loop | Not implemented. `smol_pwm_fill()` has no body. The bridge PWM uses fixed values from `_ram.pwm_cc_loop_bufs`. | `smol_servo_pwm.c:88` |
| Current conversion (ADC volts to amperes) | Draft, does not compile, not in the build | `smol_servo_isense.c` |
| Estimator (model, filter) | Draft, not in the build | `smol_servo_model.c` |
| Controller | Not started | — |
| Modes, startup sequence | Not started | — |
| Current sense zero calibration | Implemented and tested at 20 V. It is the first phase of `smol_servo_loop_start()`. See section 7.1. | `smol_servo_loop.c` |
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
- The ADC trigger is 150 clocks (1 µs) early, because a conversion takes 2 µs. The sample center is then on the PWM counter 0 or TOP.
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
- At the end of the handler, `assert_fast` checks that the loop IRQ is not pending again. If it is pending, the loop took more than one period.

### 3.4 Time budget

- 7500 clocks per loop at 150 MHz.
- The ADC IRQ (every 4 samples, 60 kHz) and the PWM IRQ (60 kHz) use part of this time. Their cost is not measured yet. GPIO 8, 9, 12 and 13 can show the IRQ times on a scope (`DEBUG_*_WITH_GPIO`).
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

- `smol_pwm_fill(pwm_values[3][3], starting_phase)` must write the PWM values for the next 3 PWM periods into `_ram.pwm_cc_loop_bufs`. The loop calls it with `starting_phase = 2`. The function has no body now.
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
5. **PWM.** Duty = U_phase / U_bus, limited to the range that keeps the valid current sample (section 3.1). Predict the rotor angle to the middle of the PWM period that applies the duty (latency 83 µs, section 3.3). Correct for the dead time (`TMC_BBMCLKS`, 42 ns per clock).

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

- `smol_servo_loop_start()` (core 0) refuses the start while the TMC6200 is enabled. It sends a request to core 1, then starts MAG, PWM and ADC. A second call does only a new calibration.
- The loop IRQ (core 1) skips 200 loops (10 ms), then adds 4096 loops (205 ms) of the 6 samples. It adds the difference to `VSENSE_NOMINAL_ZERO_V`, so that the float sums keep their resolution. Core 1 writes all sums. Core 0 reads them only after the done flag.
- Core 0 (`smol_servo_loop_idle_core0()`) calculates the mean and the standard deviation, checks the limits and logs the result. Core 1 cannot log here, because `printf` is in flash.
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

1. `smol_pwm_fill()` has no body. The loop cannot change the PWM.
2. `smol_servo_isense.c` and `smol_servo_model.c` do not compile and are not in the build.
3. `smol_servo_loop_start()` is commented out in `_core1_init`. The console command `s` starts the peripherals from core 0. The interrupts still run on core 1, but the start is outside the mode control of core 1.
4. `smol_adc_vbus()` reads 0.000 because the ADC does not run.
5. No mode state machine and no connection between the loop (core 1) and the driver enable (core 0).
6. The documentation in `doc/n17-servo-design/n17-servo-design.tex` describes the STM32G4 and DRV8323 hardware (TIM2, 50 kHz, low-side shunts). `doc/smol-servo-modeling/smol-servo-modeling.tex` has only an introduction.

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
6. **Dead time.** BBMCLKS after a scope measurement. The reset default 4 (168 ns) is in use now.

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

