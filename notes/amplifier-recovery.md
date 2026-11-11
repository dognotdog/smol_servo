# Current sense amplifier error (TMC6200): measurement and model

Date: 2026-11-10. Board: smol-servo RP2350 + TMC6200 driver board. VBUS 20 V unless stated. Bridge PWM: phase-correct, 60 kHz, CC_FULL 1250 (duty = cc / 1250, 1 count = 6.67 ns of each half of the high time). Loop: 20 kHz, 3 PWM periods.

Scale: gain 20, shunt 5 mΩ. 1 A = 100 mV at the ADC = 5 mV at the amplifier input. 1 mA = 0.1 mV at the ADC. A positive current makes the ADC voltage go down.

## 1. Summary

- **ADC sample instant.** The ADC is a point sampler. With the trigger advance of 60 counts, the hold is at about counter 0 and TOP (section 3.1).
- **Two samples per PWM period.** Sample 0 (counter 0) is in the high time of the terminal. Sample 1 (TOP) is in the low time. Sample 0 uses the cc of the previous period, sample 1 the cc of this period.
- **Edge responses.** Each terminal edge causes a 30 MHz ringing (about 0.1 µs), then a deep dip (rising edge: −1.8 to −2.6 A at about 1 µs, falling edge: −6.7 to −8.3 A at about 1.5 µs). The response settles after 2.4–5 µs (section 3.2).
- **Static offsets.** The high state reads +1.5 to +3 A. The low state after a falling edge reads +0.15 to +0.45 A (a tail of about 20 µs).
- **Bridge output limits.** A pulse of 22 counts (293 ns) or longer works in all periods. Decision: zero current at 50 % duty, switching range cc 25–1225 (section 3.4).
- **Low-state drift.** The low-state reading drifts by up to about 1 A with the switching history (τ about 3 s and 40 s). The drift also depends on the duty. The high-state reading is stable (section 3.7). The cause is not known, and it will not be measured. The model tracks the drift.
- **Supply voltage.** One table is sufficient for VBUS ≥ 15 V (section 3.5).
- **Motor.** A connected, stationary motor gives the same result as no motor (section 3.6).
- **Calibration.** `id_cal` measures the table in one 22 s run. The current table has knots every 16 cc, with the interpolation error and the per-loop standard deviation for each interval (sections 4 and 5).

## 2. Test commands

Code: `firmware_rpico/src/smol_motor_id.c` and `smol_console.c`. In both tests, all 3 terminals get the same duty, so no current flows. The rotor must not turn. The tests have no abort check of their own. The overcurrent check of the loop stays active.

### `id_rec [lo_cc] [hi_cc] [step_cc] [loops]`

Defaults `0 1250 1 1000`. One run for each cc, with a stop between the runs:

1. Settle: 200 loops at duty 0 (all 3 low sides on). The mean of the second half is the baseline of the run.
2. `loops` loops at the cc. Core 1 adds the samples minus the baseline in 9 time bins of equal length.
3. Output for each cc: `mean` and `sigma` of the last bin, `drift` (bin 1 − bin 8), and `base` (the baseline relative to ZERO_CAL).

With `lo = hi` and `loops` 20000, one run is 1 s. This is the hold test of section 3.7.

### `id_cal [lo_cc] [hi_cc] [loops_per_cc] [warm_s] [warm_cc]`

Defaults `25 1225 100 10 625`. One run without a stop:

1. Warmup: `warm_s` seconds of switching at `warm_cc`.
2. Start baseline: 200 loops at duty 0 (the second half is used).
3. Up sweep: cc lo..hi, `loops_per_cc` loops at each cc.
4. Down sweep: cc hi..lo, the same.
5. End baseline: 200 loops at duty 0.

- After each cc change, the first 3 loops are not used (output delay).
- Core 1 adds the sum and the sum of squares for each cc and direction. All values are relative to ZERO_CAL.
- After the run, core 0 prints the two baselines and one line per cc: the up mean, the down mean, and `sd`, the standard deviation of one loop (up and down pooled).
- With the defaults, the run takes 22 s. The printout takes another 13 s (about 90 lines per second).
- The sums use 120 kB of static RAM.

## 3. Measurements

τ is the time from the terminal edge to the ADC hold. The terminal edge comes about 0.1 µs later than PWM edge + BBM (168 ns). With the advance of 60 counts:

- Sample 0: τ_rise = D_prev·T/2 − 0.05 µs − BBM − 0.1 µs.
- Sample 1: τ_fall = (1 − D)·T/2 − 0.05 µs − BBM − 0.1 µs.

The calibration table uses cc directly, so it does not depend on this formula.

### 3.1 ADC sample instant

- Scope cross-check with 18 fixed duties (SOA/SOB/SOC, GPIO9 as the counter 0 reference). The scope voltage at one point per window agrees with the ADC mean (rms difference about 225 mA, errors from −8.3 A to +3.5 A). A mean over the 2 µs conversion window fits much worse (rms 1.1 A). Figures `rec-sweep-xcheck.png` and `rec-response-vs-tau.png`.
- With the old advance of 150 counts, the hold was 0.65 µs before counter 0 and TOP. Thus the ADC holds the input about 0.35–0.45 µs after the trigger. The RP2350 datasheet does not give this instant.
- **Change (commit 0a5286d):** `SMOL_SERVO_ADC_ADVANCE_COUNT` = `SMOL_SERVO_ADC_CENTER_COUNT` (150) − `SMOL_SERVO_ADC_HOLD_ADJUST_COUNT` (90) = 60 counts. A sweep after the change agrees with the old sweep moved by +0.58 to +0.60 µs (rms 29–82 mA). The hold is now at about counter 0 − 0.05 µs and TOP − 0.05 µs. Figure `rec-sweep-advance60.png`.
- The scope values are 9–13 % larger than the ADC values on the plateaus. The cause is not known (probe calibration, ADC reference, or scaling).

### 3.2 Response to the terminal edges

Values before the trigger change, from the 1 % sweep (τ values with the old formula, without the 0.1 µs correction):

| | A | B | C |
|---|---|---|---|
| Rising edge: first dip | −2.57 A at 1.02 µs | −1.78 A at 0.93 µs | −1.97 A at 0.93 µs |
| Rising edge: settled (within 10 % of the high level) after | 2.7 µs | 2.35 µs | 2.4 µs |
| High level while switching (τ 3–5 µs) | +1.45 A | +2.97 A | +2.47 A |
| High level at D = 1 (no edges) | +0.76 A | +1.35 A | +1.40 A |
| Falling edge: first dip | −8.33 A at 1.43 µs | −6.69 A at 1.52 µs | −7.07 A at 1.52 µs |
| Falling edge: bump | +1.3 A at 3.3 µs, below 0.25 A at 4.8 µs | none | none |
| Falling edge: settled after | about 5 µs | 2.8 µs | 2.8 µs |
| Low level while switching (τ 5–7 µs) | +0.15 to +0.45 A | +0.27 to +0.32 A | +0.25 to +0.27 A |
| Noise (sigma), terminal low | 85–150 mA | 30 mA | 25–37 mA |
| Noise (sigma), terminal high | 11–13 mA | 16–22 mA | 22 mA |

- **The two edges do not give opposite responses.** Both start with a negative dip, and the falling dip is 3–4 times deeper. Thus the error is not a linear common-mode response. The amplifier probably goes into a large-signal state (slew limit or saturation) after each 20 V common-mode step.
- **The load has no effect** on the edge response. The pulse test with and without the motor gives the same ringing and tail (`recovery-motor-vs-nomotor.png`).
- **The low tail** after a falling edge (+0.15 to +0.45 A) decays over about 20 µs in the pulse test. It is longer than one PWM period, so the tails of several periods add.
- **The 230 kHz noise on A** is present only when the A low side is on (85–150 mA with the terminal low, 11–13 mA with the terminal high). It is not present when VDD is in dropout (section 3.5). Thus the LT3990 buck (U5) is the likely source. Figures `soa-ripple-230khz.png` and `soa-ripple-single.png`.
- Full sweep in steps of 1 cc: figures `rec-cc-full.png`, `rec-cc-low.png` and `rec-cc-high.png`.

### 3.3 Ringing at about 60 counts from each end

Sample 0 at cc 52–65 (−0.4 A) and sample 1 at cc 1185–1198 (+0.5 A), both with a high sigma. Figure `soa-edge-zoom.png`.

- This is the 30 MHz ringing at the terminal edge, sampled by the ADC. On the scope, SOA rings for about 0.1 µs after each terminal edge.
- The sigma is high only at these cc values, because a few ns of jitter between the edge and the hold gives a large scatter. In the `id_cal` data, the per-loop sd there is up to 3.9 A.
- The sign is opposite for the rising and falling edges. This agrees with a linear coupling of the edge (dV/dt). It is the same with and without the motor.
- At cc 0–20 (no high-side turn-on), sample 0 has a ±0.2 A pattern with a period of 5 counts (33 ns). That agrees with the 30 MHz ringing.

### 3.4 Bridge output limits (scope on terminal A)

Scope ch4 (10:1) on terminal A, 20 V, one `id_rec <cc> <cc> 10 20000` run for each value. Figures `term-a-min-pulse.png` (motor connected) and `term-a-min-pulse-nomotor.png`.

| cc | Input pulse | Terminal A |
|---|---|---|
| 0, 15 | high 0 / 200 ns | Stays at 0 V. The high side does not turn on. |
| 19 | high 253 ns | Stays at 0 V in most periods. Some periods have one high pulse. |
| 20 | high 267 ns | High pulse in some periods only. |
| 25 | high 333 ns | High pulse in each period. |
| 1229 | low 280 ns | Correct: 20 V, low for about 340 ns in each period. |
| 1230 | low 267 ns | Irregular. In some periods, it goes low, then rises slowly back to 20 V over about 10 µs. |
| 1231 | low 253 ns | 14–18 V. |
| 1232–1240 | low 240–133 ns | **10–13 V.** A sawtooth. The low side never turns on, and the high side does not turn on fully again. |
| 1249 | low 13 ns | 13–17 V, slow sawtooth. |
| 1250 | static high | **17 V, not 20 V.** |

Exact limit (no motor, 315 periods at each value):

| Pulse | Low end: periods with a high pulse | High end: periods with a low pulse | High end: time at ≥ 19 V |
|---|---|---|---|
| 19 counts (253 ns) | — | cc 1231: 20 % | 0 % (floating) |
| 20 counts (267 ns) | cc 20: 50 % | cc 1230: 52 % | 56 % |
| 21 counts (280 ns) | cc 21: 83 % | cc 1229: 85 % | 97.6 % |
| 22 counts (293 ns) | cc 22: 100 % | cc 1228: 100 % | 97.8 % |
| 23–26 counts | cc 23–26: 100 % | cc 1224–1227: 100 % | 97.8 % |

- The limit is the same at both ends: 22 counts (293 ns = BBMCLKS 4 (168 ns) + about 125 ns). The TMC6200 datasheet (§3.4) gives the effective BBM as the gate slope time plus tBBM. At 9 V, the threshold is different.
- After a low input that is too short, the high side is not driven on again. The terminal floats near the gate voltage minus the MOSFET threshold (source follower). At cc 1232–1249, the samples read like duty 0.
- The motor does not set the floating level (the same within 0.2 V without the motor).
- At cc 1250 (static high), the terminal is only at 17 V, with and without the motor. VCP and CU were not measured. On this board, VSA and 12VOUT are tied to VDD (10.2 V from the LT3990), not to VS.
- **Decision:** the zero of each phase is at 50 % duty. The switching range is cc 25–1225 (22 counts plus margin at each end). In this range, each channel always has one settled sample.

### 3.5 Supply voltage

Sweeps at 5 contract voltages (`pd_req`), no motor, advance 60. Figure `rec-sweep-vbus.png`.

| VBUS | VDD | B0 high level (D 0.5) | B1 low level (D 0.3) | B1 falling dip | A1 noise (sigma, terminal low) |
|---|---|---|---|---|---|
| 9 V (fixed) | 7.8 V | +3.42 A | +0.03 A | −7.01 A at D 0.78 | 27 mA |
| 10 V (PPS) | 8.6 V | +3.38 A | +0.05 A | −7.00 A at D 0.78 | 28 mA |
| 12 V (PPS) | 9.9 V | +3.14 A | +0.23 A | −6.94 A at D 0.78 | 61 mA |
| 15 V (fixed) | 10.2 V | +2.99 A | +0.33 A | −6.78 A at D 0.79 | 62 mA |
| 20 V (fixed) | 10.2 V | +2.96 A | +0.32 A | −6.78 A at D 0.79 | 60 mA |

- The error does not scale with VBUS. 15 V and 20 V give the same curves (rms difference 17–61 mA, except A1). The change occurs between 10 V and 15 V, where the LT3990 (VDD, also the TMC6200 analog supply VSA) leaves dropout.
- With VDD in dropout: the terminal-high offset is 0.4–0.5 A larger, the low tail is about 0, the falling-edge response is 0.1–0.15 µs later, and the A 230 kHz noise is not present.
- One table is sufficient for VBUS ≥ 15 V. Below that, the table must depend on the supply voltage.

### 3.6 Motor connected

Sweep at 20 V with the motor connected and stationary. Figure `rec-sweep-motor.png`.

- The curves agree with the no-motor sweep (time shift 0.00 ± 0.01 µs, rms 14–29 mA in the settled range, the same noise).
- Thus the calibration can run with the motor connected, if the rotor does not turn. A short calibration at boot, at the actual VBUS, is possible.

### 3.7 Low-state drift

**Hold test at cc 625** (figure `hold-625.png`, data `hold-625.npz`). After a long rest, 1 s `id_rec` runs at cc 625 for 180 s (gap about 0.1 s between runs), then the driver off for 12 s, then 60 s of runs. All values relative to ZERO_CAL:

| Change in 180 s | A | B | C |
|---|---|---|---|
| Sample 0 (terminal high) | −30 mA | −42 mA | −44 mA |
| Sample 1 (terminal low) | +615 mA | +703 mA | +820 mA |
| Baseline (all low sides on, no switching) | +663 mA | +843 mA | +983 mA |

- At the ADC, the baseline change is −66 to −98 mV (3.3–4.9 mV at the amplifier input).
- The first run after the long rest has a baseline of −16 to −72 mA, that is, almost ZERO_CAL.
- Fit of the baseline with 2 exponentials: τ 2.7–3.4 s and τ 36–39 s (rms residual 12–13 mA).
- After 12 s with the driver off, the baseline is 52–54 % back to the rested value.
- Inside each 1 s run, sample 0 changes by only 0–4 mA after the first 0.1 s. In 50 ms runs, a fast change of 8–38 mA occurs in the first 0.1 s (figure `slow-component.png` (c)).

**Dependence on the duty.** In a sweep of separate runs after a 10 s warmup (data `cal32.npz`), the baseline increased from about 380/500/540 mA (A/B/C) at cc 25–125 to 530/720/780 mA at cc 425–525. Then it decreased to 250/365/405 mA at cc 1225. Thus the drift increases when the low side is on for a long time, and it decreases at high duty. It does not depend only on the switching time.

**Effect on the earlier sweeps.** In separate `id_rec` runs, E = sample − baseline, and the baseline changes with the history. This caused the step of 0.15–0.22 A at cc 1050 between two sweep runs (figure `slow-component.png` (a), (b)).

**Up and down sweep (`id_cal`).** The mean of the up and the down value cancels a drift that is linear in time. Results:

- Sample 0 above cc 300: up − down is −4 mA (median). The high state does not drift.
- Sample 1, and sample 0 below cc 300: up − down goes from −150 mA (cc 25) to +75 mA (cc 750) and −120 mA (cc 1000). A drift linear in time would give a straight line. Thus the mean of up and down does not cancel all of the drift. The remaining error in sample 1 is estimated at some tens of mA. It is not measured.

## 4. Model for the implementation

For channel k and the switching range cc 25–1225, with Z_k the ZERO_CAL value and G·R the scale:

```
sample 0 (counter 0, terminal high):  y_k0 = Z_k + G·R·i_k + E_k0[cc_prev]
sample 1 (TOP, terminal low):         y_k1 = Z_k + d_k(t) + G·R·i_k + E_k1[cc]
```

- **E_k0** is relative to ZERO_CAL. The high state does not drift.
- **E_k1** is relative to the static low level (the mean of the start and end baselines of `id_cal`).
- **d_k(t)** is the low-state drift (section 3.7): up to about 1 A, τ about 3 s and 40 s, and dependent on the duty. The loop must track it, or must use sample 0. The drift is similar in A, B and C (within about ±20 %). Thus the condition i_A + i_B + i_C = 0 can find most of it.
- **cc_prev** is the cc of the previous PWM period, because the rising edge for sample 0 is in that period.

**Table.** Knots every 16 cc, cc 25 to 1225 (76 knots). The knot values are a least-squares fit of a piecewise-linear function to the per-cc mean of the up and down sweep. In integers:

```
d = cc − 25;  i = d >> 4;  f = d & 15;
E = E[i] + (((E[i+1] − E[i]) * f) >> 4);     // i + 1 ≤ 75 for cc ≤ 1225
```

Data: `int16_t E[3][2][76]` in mA, 912 bytes.

**Firmware (`smol_isense_cal.c/.h`).** At the end of each `id_cal`, core 0 fits the table on the board (least squares with hat functions, the same method as the host script) and writes it to `/isense_cal.bin` on littlefs:

- File: a 52-byte header (cc range, gain, VBUS, the `id_cal` parameters, the start and end baselines), E, err and sd per interval, then a CRC. For 76 knots it has 2768 bytes.
- `main()` loads it at boot after the config. `cfg_load` loads it again. If the gain in the file is not the gain in the config, no table is in use.
- RAM: the file image is 2908 bytes (80 knots maximum), and the fit uses 1280 bytes. The loop needs only E.
- Console: `cal_show` prints the header and the knots, and `cal_load` reads the file again.
- Check: on the same `id_cal` data (`cal5.log`), the board table and the host fit agree within 1 mA (E) and 2 mA (err, sd).
- The loop does not apply the table yet. The inline function `smol_isense_cal_e_ma(p, j, cc)` gives the value.

**Sample selection.** Use one contiguous range for each sample. The ranges come from the timing sensitivity: under load, the terminal edge comes up to one BBM (25 counts) earlier than at zero current. The sensitivity is the largest change of E in that window. The ranges are from the `id_rec` sweep in steps of 1 cc (figure `cal-table.png`):

| | Sample 1 (TOP) | Sample 0 (counter 0) | Both: weight of sample 0 | Sensitivity > 50 mA |
|---|---|---|---|---|
| A | cc 25–514 | cc 449–1225 | 0.94 (σ 12 / 51 mA) | cc 31–95, 324–407 (max 201 mA) |
| B | cc 25–740 | cc 442–1225 | 0.74 (σ 19 / 31 mA) | cc 31–70 (max 114 mA) |
| C | cc 25–734 | cc 433–1225 | 0.66 (σ 20 / 27 mA) | cc 27–56 (max 122 mA) |

- Where both samples are valid, use the inverse-variance mean (weight of sample 0 in the table).
- A, cc 324–407: sample 0 is not settled yet, and sample 1 has the A low-state ripple.
- Isolated flat points inside a transient (for example the bottom of the rising dip at cc ≈ 170) are flat only by chance. Do not use them.

**Error estimate.** For one sample in the loop: err + 2·sd from the table in section 5. In the used ranges:

| Range | err | sd (one loop) |
|---|---|---|
| Sample 0, cc ≥ 440 | 1–3 mA | 12–18 mA |
| B1, C1, cc ≤ 740 | 4–9 mA | 27–37 mA |
| A1, cc ≤ 510 | 6–80 mA | 46–126 mA (A low-state ripple) |

Outside the used ranges, err is up to 986 mA and sd up to 3.9 A (the ringing near both ends), and sd is 100–270 mA on the slopes of the deep dip of sample 1 (cc 800–1100).

**Limits of the model.**

- The tails of the edge responses (about 0.3 A, 20 µs) add over several periods. After a fast duty step, the table error is about the tail size for about 20 µs.
- The remaining drift error in sample 1 after the up/down mean (some tens of mA, not measured).
- The table is for VBUS ≥ 15 V.
- The response to the edges of the other phases is not in the model (decision: ignore it, because its effect looks much smaller).

## 5. Current calibration table (16 cc)

`id_cal` with the defaults, after a long time with the bridge off. Log `cal4.log`. Data `notes/scope/cal16.npz`. Figures `notes/scope/cal16-table.png` and `notes/scope/cal16-curves.png`.

- Sample 0 is relative to ZERO_CAL. Sample 1 is relative to the mean baseline.
- Baselines (A/B/C, sample 0): start 407/622/630 mA, end 518/739/755 mA.
- **err**: the max |measurement − interpolation| from this knot to the next knot.
- **sd**: the max standard deviation of one loop in the same interval. The standard error of the table value itself is sd / √194 (median 1–6 mA).

Values in mA (1 mA = 0.1 mV at the ADC):

| cc | A0 | err | sd | A1 | err | sd | B0 | err | sd | B1 | err | sd | C0 | err | sd | C1 | err | sd |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 25 | 383 | 157 | 161 | 118 | 24 | 56 | 615 | 150 | 73 | 91 | 5 | 30 | 666 | 122 | 68 | 95 | 8 | 31 |
| 41 | 688 | 179 | 582 | 205 | 21 | 60 | 905 | 254 | 767 | 149 | 5 | 30 | 860 | 164 | 863 | 160 | 7 | 29 |
| 57 | 249 | 262 | 890 | 282 | 11 | 61 | 278 | 918 | 3851 | 207 | 7 | 28 | 479 | 859 | 3830 | 195 | 7 | 28 |
| 73 | 673 | 169 | 556 | 364 | 12 | 62 | 1378 | 913 | 3019 | 223 | 8 | 31 | 1430 | 986 | 3094 | 196 | 5 | 29 |
| 89 | 585 | 44 | 172 | 406 | 13 | 54 | 732 | 227 | 739 | 233 | 6 | 30 | 732 | 209 | 649 | 200 | 6 | 29 |
| 105 | 265 | 28 | 114 | 412 | 10 | 48 | 453 | 115 | 399 | 236 | 8 | 32 | 454 | 99 | 378 | 208 | 7 | 30 |
| 121 | -334 | 18 | 119 | 434 | 6 | 46 | -57 | 19 | 120 | 241 | 7 | 31 | -77 | 20 | 124 | 207 | 8 | 29 |
| 137 | -1030 | 34 | 117 | 445 | 6 | 48 | -612 | 15 | 99 | 242 | 6 | 30 | -656 | 19 | 106 | 209 | 6 | 30 |
| 153 | -1666 | 73 | 112 | 457 | 34 | 54 | -1037 | 13 | 46 | 246 | 9 | 31 | -1146 | 21 | 50 | 214 | 7 | 30 |
| 169 | -1948 | 50 | 96 | 484 | 34 | 69 | -1276 | 23 | 33 | 242 | 9 | 30 | -1433 | 17 | 35 | 214 | 5 | 28 |
| 185 | -2277 | 58 | 92 | 421 | 26 | 76 | -1285 | 15 | 36 | 250 | 8 | 30 | -1508 | 15 | 33 | 211 | 5 | 28 |
| 201 | -2215 | 38 | 89 | 446 | 16 | 73 | -1089 | 12 | 45 | 250 | 5 | 32 | -1368 | 12 | 44 | 218 | 5 | 30 |
| 217 | -1999 | 9 | 72 | 455 | 11 | 68 | -757 | 13 | 61 | 249 | 6 | 30 | -1092 | 10 | 54 | 218 | 9 | 29 |
| 233 | -1679 | 10 | 70 | 460 | 9 | 61 | -318 | 10 | 60 | 255 | 8 | 30 | -694 | 11 | 55 | 223 | 6 | 30 |
| 249 | -1307 | 10 | 69 | 439 | 24 | 76 | 167 | 4 | 59 | 258 | 5 | 31 | -262 | 4 | 59 | 225 | 7 | 28 |
| 265 | -900 | 16 | 56 | 416 | 15 | 71 | 657 | 10 | 59 | 261 | 7 | 31 | 192 | 7 | 55 | 229 | 5 | 28 |
| 281 | -495 | 7 | 56 | 372 | 25 | 91 | 1134 | 5 | 56 | 259 | 7 | 31 | 634 | 7 | 53 | 232 | 6 | 29 |
| 297 | -107 | 4 | 50 | 357 | 23 | 77 | 1569 | 6 | 48 | 267 | 6 | 31 | 1046 | 4 | 48 | 230 | 5 | 28 |
| 313 | 261 | 10 | 44 | 378 | 42 | 65 | 1957 | 4 | 42 | 267 | 7 | 30 | 1418 | 4 | 42 | 233 | 5 | 28 |
| 329 | 592 | 6 | 34 | 203 | 80 | 112 | 2292 | 6 | 38 | 276 | 7 | 31 | 1742 | 6 | 38 | 237 | 4 | 30 |
| 345 | 841 | 4 | 30 | 203 | 41 | 126 | 2570 | 5 | 32 | 289 | 7 | 32 | 2010 | 4 | 32 | 247 | 6 | 30 |
| 361 | 1061 | 4 | 26 | 252 | 52 | 116 | 2799 | 3 | 27 | 287 | 4 | 34 | 2235 | 3 | 28 | 246 | 6 | 30 |
| 377 | 1225 | 4 | 22 | 161 | 46 | 110 | 2973 | 4 | 24 | 301 | 6 | 32 | 2410 | 3 | 24 | 253 | 6 | 29 |
| 393 | 1370 | 4 | 20 | 275 | 28 | 106 | 3115 | 4 | 20 | 301 | 6 | 32 | 2552 | 3 | 22 | 250 | 6 | 28 |
| 409 | 1473 | 2 | 18 | 242 | 19 | 102 | 3217 | 3 | 18 | 306 | 5 | 33 | 2658 | 2 | 19 | 256 | 5 | 27 |
| 425 | 1551 | 1 | 14 | 220 | 9 | 78 | 3292 | 3 | 17 | 309 | 7 | 32 | 2736 | 2 | 19 | 255 | 5 | 28 |
| 441 | 1604 | 2 | 14 | 205 | 10 | 56 | 3345 | 2 | 16 | 319 | 5 | 34 | 2793 | 2 | 17 | 263 | 5 | 28 |
| 457 | 1638 | 2 | 14 | 222 | 6 | 51 | 3376 | 2 | 15 | 327 | 5 | 35 | 2830 | 2 | 17 | 259 | 5 | 28 |
| 473 | 1660 | 2 | 14 | 233 | 10 | 53 | 3397 | 2 | 15 | 333 | 7 | 34 | 2856 | 1 | 17 | 263 | 5 | 28 |
| 489 | 1673 | 1 | 13 | 236 | 15 | 62 | 3406 | 2 | 16 | 347 | 4 | 34 | 2869 | 1 | 17 | 264 | 6 | 29 |
| 505 | 1680 | 3 | 13 | 192 | 10 | 58 | 3409 | 2 | 16 | 356 | 7 | 34 | 2875 | 2 | 17 | 273 | 5 | 28 |
| 521 | 1679 | 3 | 13 | 183 | 9 | 46 | 3409 | 2 | 16 | 378 | 9 | 34 | 2879 | 3 | 17 | 286 | 7 | 28 |
| 537 | 1674 | 1 | 13 | 324 | 19 | 72 | 3407 | 2 | 16 | 383 | 4 | 35 | 2879 | 2 | 17 | 290 | 5 | 28 |
| 553 | 1675 | 1 | 13 | 397 | 16 | 99 | 3405 | 1 | 15 | 384 | 5 | 36 | 2878 | 2 | 16 | 292 | 6 | 31 |
| 569 | 1672 | 2 | 14 | 456 | 23 | 99 | 3401 | 2 | 15 | 403 | 5 | 37 | 2877 | 2 | 16 | 299 | 7 | 29 |
| 585 | 1669 | 1 | 14 | 553 | 25 | 134 | 3398 | 1 | 16 | 408 | 5 | 37 | 2875 | 2 | 17 | 307 | 9 | 29 |
| 601 | 1666 | 2 | 14 | 617 | 18 | 128 | 3397 | 2 | 16 | 418 | 6 | 35 | 2874 | 2 | 17 | 324 | 4 | 30 |
| 617 | 1666 | 1 | 13 | 718 | 15 | 125 | 3397 | 2 | 16 | 425 | 6 | 30 | 2874 | 1 | 17 | 337 | 8 | 27 |
| 633 | 1665 | 1 | 14 | 869 | 16 | 118 | 3396 | 1 | 14 | 440 | 5 | 29 | 2873 | 2 | 17 | 351 | 8 | 28 |
| 649 | 1666 | 2 | 14 | 980 | 15 | 56 | 3397 | 3 | 15 | 463 | 6 | 31 | 2875 | 2 | 18 | 361 | 6 | 28 |
| 665 | 1666 | 3 | 13 | 1169 | 43 | 124 | 3398 | 2 | 14 | 493 | 4 | 28 | 2875 | 2 | 16 | 385 | 6 | 28 |
| 681 | 1668 | 1 | 13 | 1079 | 25 | 127 | 3400 | 1 | 15 | 497 | 5 | 30 | 2877 | 2 | 18 | 394 | 6 | 27 |
| 697 | 1671 | 1 | 13 | 1209 | 30 | 170 | 3402 | 1 | 16 | 502 | 8 | 32 | 2879 | 2 | 17 | 393 | 4 | 28 |
| 713 | 1674 | 1 | 13 | 1259 | 30 | 175 | 3406 | 2 | 16 | 495 | 8 | 34 | 2882 | 2 | 18 | 383 | 6 | 29 |
| 729 | 1679 | 1 | 13 | 1303 | 28 | 173 | 3409 | 2 | 16 | 472 | 7 | 34 | 2886 | 3 | 18 | 348 | 7 | 30 |
| 745 | 1681 | 3 | 13 | 1386 | 91 | 173 | 3412 | 2 | 17 | 422 | 4 | 34 | 2888 | 2 | 18 | 290 | 9 | 30 |
| 761 | 1683 | 2 | 12 | 1282 | 28 | 157 | 3415 | 2 | 15 | 328 | 4 | 33 | 2890 | 2 | 17 | 202 | 5 | 32 |
| 777 | 1688 | 2 | 12 | 1295 | 38 | 100 | 3419 | 2 | 15 | 197 | 8 | 39 | 2895 | 2 | 18 | 76 | 8 | 36 |
| 793 | 1692 | 1 | 12 | 910 | 27 | 123 | 3423 | 1 | 16 | 18 | 7 | 46 | 2899 | 1 | 18 | -110 | 11 | 44 |
| 809 | 1696 | 1 | 13 | 600 | 28 | 120 | 3429 | 2 | 16 | -228 | 8 | 50 | 2904 | 2 | 17 | -358 | 9 | 52 |
| 825 | 1700 | 1 | 12 | 273 | 14 | 102 | 3434 | 2 | 16 | -569 | 8 | 68 | 2908 | 2 | 18 | -708 | 13 | 61 |
| 841 | 1705 | 2 | 13 | -234 | 28 | 121 | 3437 | 1 | 16 | -1012 | 8 | 69 | 2911 | 2 | 17 | -1140 | 11 | 74 |
| 857 | 1709 | 2 | 12 | -856 | 59 | 138 | 3442 | 2 | 15 | -1533 | 11 | 84 | 2916 | 3 | 18 | -1678 | 12 | 78 |
| 873 | 1712 | 2 | 12 | -1633 | 38 | 128 | 3446 | 2 | 15 | -2174 | 14 | 90 | 2920 | 2 | 16 | -2314 | 13 | 88 |
| 889 | 1715 | 1 | 11 | -2743 | 31 | 215 | 3448 | 2 | 15 | -2880 | 10 | 96 | 2922 | 2 | 17 | -3033 | 6 | 96 |
| 905 | 1719 | 1 | 12 | -3621 | 48 | 253 | 3454 | 2 | 15 | -3665 | 7 | 104 | 2926 | 2 | 16 | -3837 | 10 | 104 |
| 921 | 1722 | 1 | 13 | -4710 | 23 | 244 | 3456 | 1 | 15 | -4513 | 10 | 99 | 2930 | 1 | 16 | -4702 | 9 | 104 |
| 937 | 1726 | 1 | 13 | -5891 | 64 | 190 | 3461 | 1 | 15 | -5356 | 12 | 94 | 2932 | 2 | 16 | -5580 | 14 | 100 |
| 953 | 1726 | 2 | 13 | -7083 | 91 | 210 | 3461 | 3 | 15 | -6135 | 12 | 86 | 2934 | 3 | 16 | -6405 | 10 | 91 |
| 969 | 1732 | 2 | 13 | -7795 | 39 | 195 | 3467 | 1 | 15 | -6792 | 23 | 68 | 2939 | 2 | 15 | -7116 | 22 | 70 |
| 985 | 1732 | 1 | 13 | -8650 | 60 | 131 | 3467 | 2 | 16 | -7194 | 34 | 35 | 2940 | 2 | 16 | -7583 | 29 | 43 |
| 1001 | 1735 | 2 | 13 | -8922 | 108 | 130 | 3469 | 2 | 15 | -7220 | 45 | 84 | 2942 | 2 | 15 | -7692 | 44 | 76 |
| 1017 | 1742 | 1 | 12 | -8663 | 114 | 143 | 3475 | 2 | 15 | -6736 | 47 | 151 | 2947 | 3 | 16 | -7300 | 49 | 142 |
| 1033 | 1744 | 1 | 13 | -7675 | 38 | 210 | 3477 | 2 | 15 | -5639 | 53 | 214 | 2950 | 2 | 17 | -6284 | 55 | 209 |
| 1049 | 1744 | 2 | 12 | -6118 | 63 | 253 | 3478 | 1 | 16 | -3923 | 41 | 244 | 2951 | 1 | 16 | -4631 | 41 | 249 |
| 1065 | 1750 | 1 | 13 | -3931 | 24 | 267 | 3483 | 2 | 16 | -1769 | 23 | 251 | 2955 | 2 | 17 | -2506 | 21 | 256 |
| 1081 | 1754 | 1 | 13 | -1553 | 54 | 242 | 3487 | 1 | 15 | 442 | 61 | 216 | 2959 | 2 | 16 | -265 | 50 | 218 |
| 1097 | 1757 | 2 | 14 | 402 | 95 | 162 | 3490 | 2 | 15 | 2121 | 88 | 128 | 2963 | 3 | 16 | 1486 | 85 | 138 |
| 1113 | 1760 | 2 | 13 | 1334 | 104 | 49 | 3493 | 2 | 15 | 2822 | 97 | 78 | 2966 | 2 | 17 | 2276 | 105 | 90 |
| 1129 | 1767 | 2 | 14 | 1451 | 21 | 61 | 3501 | 2 | 16 | 2853 | 105 | 269 | 2973 | 3 | 18 | 2340 | 111 | 310 |
| 1145 | 1767 | 1 | 14 | 1397 | 24 | 85 | 3501 | 1 | 16 | 2798 | 162 | 432 | 2973 | 2 | 18 | 2284 | 141 | 485 |
| 1161 | 1772 | 1 | 14 | 1338 | 139 | 470 | 3505 | 2 | 16 | 2750 | 807 | 2470 | 2978 | 2 | 17 | 2243 | 877 | 2644 |
| 1177 | 1777 | 1 | 14 | 1233 | 279 | 920 | 3510 | 2 | 16 | 3097 | 774 | 3755 | 2984 | 2 | 17 | 2624 | 738 | 3771 |
| 1193 | 1780 | 3 | 14 | 1936 | 244 | 807 | 3513 | 2 | 16 | 3747 | 491 | 1347 | 2985 | 3 | 17 | 3027 | 567 | 1579 |
| 1209 | 1779 | 2 | 14 | 1261 | 123 | 21 | 3513 | 2 | 15 | 2715 | 177 | 62 | 2986 | 1 | 16 | 2205 | 140 | 67 |
| 1225 | 1780 | - | - | 1391 | - | - | 3515 | - | - | 2922 | - | - | 2988 | - | - | 2359 | - | - |

## 6. History of the table

| Step | Method | Result | Files |
|---|---|---|---|
| 1 | `id_rec` per cc, separate runs of 1000 loops | One value per cc (14.4 kB). Step of 0.15–0.22 A at the run boundary (drift). | `rec-cc-*.png`, `cal-table.png` |
| 2 | 32 cc knots, least-squares fit to step 1 | 39 knots (468 bytes), max error ≤ 24 mA except A1 (73 mA) | `cal-table-32.png` |
| 3 | 10 s warmup, then `id_rec` per cc with 2000 loops (176 s) | First table with err per interval. Showed the dependence of the drift on the duty. | `cal32-*.png`, `cal32.npz` |
| 4 | `id_cal`, up and down in one run (22 s), 32 cc | err of B1, C1 in the used range 5–14 mA | `calud-*.png`, `calud.npz` |
| 5 | `id_cal` with per-loop sd, 16 cc | The current table (section 5) | `cal16-*.png`, `cal16.npz` |

## 7. Open items

- **A model for the low-state drift d_k(t)** that the loop can track (time and duty dependence), and a check of the condition i_A + i_B + i_C = 0 for it. The cause is not known. It will not be measured (decision 2026-11-10).
- The scale difference of 9–13 % between the scope and the ADC.
- The terminal at cc 1250 is only at 17 V (VCP and CU not measured).
- The A noise in the low state (230 kHz, likely the LT3990). Not confirmed with the scope.
- Firmware: the `id_cal` sums use 120 kB of static RAM. The variance uses Σx²/n − m² in float (an error of some mA at |E| ≈ 9 A). `stop` does not end the printout.
