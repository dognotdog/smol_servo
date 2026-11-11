# Current sense amplifier recovery (TMC6200), measurement and model

Date: 2026-11-10. Board: smol-servo RP2350 + TMC6200 driver board. VBUS 20.1 V. Gain 20, shunt 5 mOhm: 10 mA per mV at SOx.

## 1. Test: `id_rec`

Command: `id_rec [lo_permille] [hi_permille] [step_permille] [loops]`, defaults `10 990 10 2000`.

- Disconnect the motor first. The test is not part of `id_auto`.
- For each duty D, the firmware does a settle phase of 200 loops at duty 0 (all 3 low sides on). The baseline is the mean of the second half.
- Then all 3 terminals get the same duty D in all PWM periods for `loops` loops. The terminal voltages are equal, so no current flows.
- Core 1 adds the samples in 9 time bins. Each duty gives 3 lines: the mean and sigma of the last bin, and the drift (bin 1 minus bin 8). All values are in mA, relative to the baseline.
- With `lo = hi`, one run also prints the baseline and all 9 bins.
- The test has no abort check. It has no differential voltage, and the amplifier errors are larger than the 1 A limit.
- Code: `MOTOR_ID_MODE_REC` in `firmware_rpico/src/smol_motor_id.c`, and the command in `smol_console.c`.

Data:

- The sweep from 0 to 100 % in 1 % steps (2000 loops each, 13 s).
- 18 fixed duties of 1 s each, with scope captures of SOA/SOB/SOC and GPIO9 as the counter-0 reference.

Figures:

- `notes/scope/rec-sweep-xcheck.png`: the error over the duty, with the scope values.
- `notes/scope/rec-response-vs-tau.png`: the same data over the time from the terminal edge, with single scope periods.

## 2. Cross-check with the scope

- The scope voltage at one point per window agrees with the ADC mean. The rms difference is about 225 mA over 3 channels × 2 samples × 17 duties. The errors span −8.3 A to +3.5 A.
- The best point is 0.65 µs before the nominal window center. The trigger is 1 µs before the center, so the ADC holds the input about 0.35 µs after the trigger. A mean over the 2 µs window fits much worse (rms 1.1 A).
- **The ADC is a point sampler at counter 0 − 0.65 µs and TOP − 0.65 µs.** The comment in `smol_servo_adc.c` assumes a sample centered on counter 0. This is not correct.
- The scope values are 9–13 % larger than the ADC values on the plateaus. The cause is not known yet: the probe calibration, the ADC reference, or the ADC scaling.

## 3. Results

τ is the time from the terminal edge to the effective ADC sample. The terminal edge comes BBM (168 ns) after the PWM edge.

- Sample 0: τ = D·T/2 − 0.65 µs − BBM, after the rising edge, with the terminal high.
- Sample 1: τ = (1 − D)·T/2 − 0.65 µs − BBM, after the falling edge, with the terminal low.

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

More results:

1. **The load has no effect.** The pulse test with and without the motor gives the same ringing and tail (`notes/scope/recovery-motor-vs-nomotor.png`). The error comes from the terminal voltage edges.
2. **The two edges do not give opposite responses.** Both start with a negative dip, and the falling dip is 3–4 times deeper. So the error is not a linear common-mode response, which would have equal size and opposite sign. The amplifier probably goes into a large-signal state, such as slew limit or saturation, after each 20 V common-mode step.
3. **The terminal state gives a static offset.** When the terminal is high, the output is 150–300 mV lower, which reads as +1.5 to +3 A. When the terminal is held high with no edges (D = 1), the offset is about half of that. So a slow component exists. Its time constant is between about 1 ms and 100 ms, because the 9 bins of a 100 ms run show no change at D = 1.
4. **The low level after a falling edge is +0.15 to +0.45 A, not 0.** In the pulse test, this tail decays over about 20 µs.
5. **A slow drift:** for D = 0.02–0.3, the low level rises by 30–70 mA during the 100 ms of a run.
6. **The 230 kHz noise on A is present only when the A low side is on.** A has a sigma of 85–150 mA with the terminal low and 11–13 mA with the terminal high. B and C do not show this. This is a clue to the source of the ripple.
7. **The duties 1 % and 99 % do not switch correctly.** At 99 %, CC = 1238, so the low time is 160 ns, which is shorter than BBM. All channels then read like duty 0, with the A noise of the low state. At 1 %, the high time is 173 ns, about one BBM. I did not look at the terminal voltage, so the exact behavior is not known. For now, do not use duties between 0 and about 2 % or between about 98 % and 100 %.

## 4. Proposed model

For channel k, the ADC sample j (0 = counter 0, 1 = TOP) in a PWM period with the duties D_prev (the period before) and D (this period):

```
y_kj = G·R·i_k + o_k + e_kj(τ_rise, τ_fall, state)
```

- o_k: the offset at duty 0 (the baseline).
- e_kj: the amplifier error. It is a sum of the responses to the past terminal edges, plus a static term for the terminal state:

```
e(t) = c_k · [terminal high at t]
     + Σ over the rising edges  r_k(t − t_edge)
     + Σ over the falling edges f_k(t − t_edge)
     + slow_k(duty history)
```

- r_k and f_k are separate responses, one for each edge direction (see result 2). Each has a fast part (the dip and the ringing, about 3 µs) and a tail of about 0.3 A that decays over about 20 µs. The fast parts do not overlap between edges if τ > 3 µs. The tails overlap, because 20 µs is longer than the PWM period.
- slow_k describes the change from the switching high level to the DC high level (result 3) and the drift (result 5). Its time constant is not measured yet.

**The edges for each sample:** the high time of phase-correct PWM is centered on counter 0. The update happens at counter 0, so:

- Sample 0 (counter 0): the rising edge is at −D_prev·T/2. It uses **the duty of the previous period**.
- Sample 1 (TOP): the falling edge is at +D·T/2. It uses **the duty of this period**.

### Use in the current loop

A full dynamic model is not necessary if each sample stays out of the fast part of the response. Then the error is a smooth function of the duty, and a table can correct it.

1. **The correction table:** `id_rec` already measures e_kj(D) for each channel and sample (101 points × 6). Store it, interpolate linearly, and subtract it. The duty in the table is D_prev for sample 0 and D for sample 1.
2. **The sample selection:** use only the sample whose τ is in the settled part:
   - Sample 0 if τ_rise ≥ 2.7 µs.
   - Sample 1 if τ_fall ≥ 2.8 µs (B, C) or ≥ 5 µs (A).
3. **Move the ADC trigger** so that the effective sample is at counter 0 and TOP. Change `SMOL_SERVO_ADC_ADVANCE_COUNT` from 150 to about 52 clocks (0.35 µs). This adds 0.65 µs to τ. Then:
   - Sample 0 is usable for D_prev ≥ 0.34.
   - Sample 1 is usable for D ≤ 0.64 (B, C) and D ≤ 0.38 (A).
   - So for each channel, at least one sample is usable over the full range of 0.02–0.98.
   - With the trigger as it is now, A has no usable sample for D between 0.30 and 0.42.
4. **Limit the duty** to 0 or 0.02–0.98, until the behavior at 1 % and 99 % is known.
5. **Limits of the table:** it does not include the slow terms or the tail sum after a fast change of the duty. The error of the table after a duty step is about the size of the tail, 0.3 A, for about 20 µs. If this is too large, add the tail as one exponential state per edge type and channel.

### Trigger change, done

- `SMOL_SERVO_ADC_ADVANCE_COUNT` is now `SMOL_SERVO_ADC_CENTER_COUNT` (150) − `SMOL_SERVO_ADC_HOLD_ADJUST_COUNT` (90) = 60 counts (0.4 µs).
- The sweep `id_rec 0 1000 10 2000` with no motor (log: `rec2`) agrees with the old sweep moved by 0.6 µs. The fitted move is +0.58 to +0.60 µs on all 6 samples. The rms difference is 29–82 mA. Without the move, it is 0.9–2.8 A.
- Figure: `notes/scope/rec-sweep-advance60.png`. The dashed curve is the old sweep moved by 0.6 µs. Near 92 %, the dashed curve has a step because of the interpolation to the 99 % point. The step is not in the data.
- The hold is now at about counter 0 − 0.05 µs and TOP − 0.05 µs. With the τ limits above, the usable ranges are:
  - Sample 0 for D_prev ≥ 0.35.
  - Sample 1 for D ≤ 0.64 (B, C) and D ≤ 0.37 (A).
  - So each channel has a usable sample for all duties from 0.02 to 0.98. For A, the overlap is small (0.35–0.37).

### Supply voltage

Sweeps `id_rec 0 1000 10 2000` with no motor and advance 60, at 5 contract voltages (`pd_req`). Figure: `notes/scope/rec-sweep-vbus.png`. VDD is the console value.

| VBUS | VDD | B0 high level (D 0.5) | B1 low level (D 0.3) | B1 falling dip | A1 noise (sigma, terminal low) |
|---|---|---|---|---|---|
| 9 V (fixed) | 7.8 V | +3.42 A | +0.03 A | −7.01 A at D 0.78 | 27 mA |
| 10 V (PPS) | 8.6 V | +3.38 A | +0.05 A | −7.00 A at D 0.78 | 28 mA |
| 12 V (PPS) | 9.9 V | +3.14 A | +0.23 A | −6.94 A at D 0.78 | 61 mA |
| 15 V (fixed) | 10.2 V | +2.99 A | +0.33 A | −6.78 A at D 0.79 | 62 mA |
| 20 V (fixed) | 10.2 V | +2.96 A | +0.32 A | −6.78 A at D 0.79 | 60 mA |

- The error does not scale with VBUS. 15 V and 20 V give the same curves (rms difference 17–61 mA, except A1). The change occurs between 10 V and 15 V.
- VDD comes from the LT3990 buck (U5, L4 33 µH) on the driver board. VDD also supplies the TMC6200 analog supply (VSA). At 9 V and 10 V, the buck is in dropout (VDD 7.8 and 8.6 V). At 12 V and above, it regulates. VBUS and VDD change together in these tests, so the data cannot separate them.
- With VDD in dropout:
  - The terminal-high offset is about 0.4–0.5 A larger, on all channels.
  - The low level after the falling edge is about 0, not +0.3 A.
  - The falling-edge response is about 0.1–0.15 µs later.
  - **The 230 kHz noise on A (terminal low) is not present.** The noise is present only when the buck switches. This makes the LT3990 the likely source of the ripple (possibly Burst Mode at light load). This is not confirmed with the scope.
- For the correction table: one table is sufficient for VBUS ≥ 15 V. Below that, the table must depend on the supply voltage.
- The 12 V run stopped at D 0.98. Then the board reset: a PPS re-request got no response, and the firmware sent a Hard Reset (log `rec_12000`).

### Motor connected

Sweep `id_rec 0 1000 10 2000` at 20 V, advance 60, with the motor connected and stationary (log `rec_motor`). Figure: `notes/scope/rec-sweep-motor.png`.

- The curves agree with the no-motor sweep `rec2`. The fitted time shift is 0.00 ± 0.01 µs.
- Difference in the settled range (the τ limits above plus one BBM): rms 14–29 mA, maximum 86 mA (A0). This is in the same range as the run-to-run scatter (15 V against 20 V: 17–61 mA).
- The largest differences (up to 140 mA) are on the steep slopes of the falling dip (D 0.82–0.85). These samples are not used.
- The noise (sigma) is the same with and without the motor.
- **Result:** `id_rec` can measure the error table with the motor connected, if the rotor does not turn. A short `id_rec` run at boot, at the actual VBUS, can thus calibrate the table, as `zero_cal` does for the offsets.

### Sweep over all PWM values

`id_rec` now steps in PWM compare counts: `id_rec [lo_cc] [hi_cc] [step_cc] [loops]`, defaults `0 1250 1 1000` (all 1251 values, about 110 s). Duty = cc / 1250. One count moves τ by 6.67 ns. Data: 20 V, advance 60, motor connected, run 1 cc 0–1049 and run 2 cc 1050–1250. Figures: `notes/scope/rec-cc-full.png`, `rec-cc-low.png`, `rec-cc-high.png`.

1. **Minimum pulse of about 250–260 ns, at both ends.**
   - Low end: the low tail in sample 1 starts only at cc ≈ 19–20 (high input 253–267 ns). Below that, the high side does not switch. Sample 0 then shows only a small ringing (±0.2 A, period about 5 counts = 33 ns).
   - High end: for cc 1232–1249 (low input 240 ns or shorter), **all 6 samples read like duty 0**, with the A noise of the low state. cc 1231 and 1230 are unstable (sigma 450–900 mA). cc 1229 and lower are normal. cc 1250 is the static high.
   - So a duty between 98.5 % and 99.9 % gives the output of duty 0, not of duty 1. The terminal voltage is not measured yet.
   - The TMC6200 datasheet (§3.4) gives the effective BBM as the gate slope time plus tBBM (BBMCLKS 4 = 168 ns). 168 ns + about 90 ns agrees with the threshold. At 9 V, cc 1238 reads high, so the threshold depends on the supply voltage.
2. **Spike after the edge:** sample 0 at cc 52–65 (τ_rise 0.13–0.22 µs) has −0.4 A with sigma up to 350 mA. Sample 1 at cc 1183–1197 (τ_fall 0.13–0.22 µs) has +0.5 A with high sigma.
3. **The response starts** at τ_rise ≈ 0.45 µs. Before that, the sample shows the level before the edge.
4. **Step at cc 1050:** in sample 0, run 2 reads 0.18–0.27 A higher than run 1. Run 1 had switched for 90 s. Between the runs, the driver was disabled for a few seconds. So the high level has a slow component with a time constant of seconds or longer, or a temperature effect. The drift inside one run (bin 1 − bin 8) is the same in both runs (20–70 mA).
5. **Duty limits for the controller:** the bridge output is correct for cc 0 and cc 25–1229 (see the terminal captures below; cc 1250 gives only 17 V). Decision: the zero of each phase is at 50 % duty, and the switching range is limited to about cc 26–1224 (about 20 counts plus margin at each end). Correction: in this range, each channel always has one settled sample. For low duties, sample 1 (TOP) is settled. For high duties, sample 0 (counter 0) is settled. The range 0.09–0.92 that I gave before is not necessary.

### Terminal voltage at the ends (scope)

Scope ch4 (10:1) on terminal A, 20 V, motor connected, `id_rec <cc> <cc> 10 20000` for each value. Figure: `notes/scope/term-a-min-pulse.png` (60 µs of one capture for each cc).

| cc | Input pulse | Terminal A |
|---|---|---|
| 0, 15 | high 0 / 200 ns | Stays at 0 V. The high side does not turn on. |
| 19 | high 253 ns | Stays at 0 V in most periods. Some periods have one high pulse. |
| 20 | high 267 ns | High pulse in some periods only. |
| 25 | high 333 ns | High pulse in each period. |
| 1229 | low 280 ns | Correct: 20 V, low for about 340 ns in each period. |
| 1230 | low 267 ns | Irregular. In some periods, it goes low, then rises slowly back to 20 V over about 10 µs. In other periods, it only drops by 2–3 V. |
| 1231 | low 253 ns | 14–18 V. It drops by 3 V in about 1 µs at each low input, then rises slowly. |
| 1232–1240 | low 240–133 ns | **10–13 V.** A sawtooth: a 3 V drop in about 1 µs at each low input, then a slow rise. The low side never turns on. The high side does not turn on fully again. |
| 1249 | low 13 ns | 13–17 V, slow sawtooth. |
| 1250 | static high | **17 V, not 20 V.** |

- The minimum input pulse at 20 V is about 255–265 ns at both ends. That is BBMCLKS 4 (168 ns) plus about 90 ns.
- After a low input that is too short, the high side is not driven on again. The terminal stays near the gate voltage minus the threshold of the high-side MOSFET. This is a source follower, which is why the voltage is not defined.
- At cc 1250 (static high), the terminal is also only at 17 V. So, with no load current, the high-side gate is not held well above VS at 100 % duty. The charge pump (VCP) and the bootstrap capacitor (CU) were not measured. Note: on this board, VSA and 12VOUT are tied to VDD (10.2 V from the LT3990), not to VS.
- The sense output follows the slow terminal ramps at cc 1230–1231 (large transients on SOA). At cc 1232–1249, the terminal is never at VS, and the samples read like duty 0.
- **Exact limit** (no motor, 20 V, 10 captures of 31 periods = 315 periods at each value):

  | Pulse | Low end: periods with a high pulse | High end: periods with a low pulse | High end: time at ≥ 19 V |
  |---|---|---|---|
  | 19 counts (253 ns) | — | cc 1231: 20 % | 0 % (floating) |
  | 20 counts (267 ns) | cc 20: 50 % | cc 1230: 52 % | 56 % |
  | 21 counts (280 ns) | cc 21: 83 % | cc 1229: 85 % | 97.6 % |
  | 22 counts (293 ns) | cc 22: 100 % | cc 1228: 100 % | 97.8 % |
  | 23–26 counts | cc 23–26: 100 % | cc 1224–1227: 100 % | 97.8 % |

  The limit is the same at both ends: 22 counts (293 ns) and longer work in all periods. The transition is from 19 to 22 counts. At cc 1229, a missed low pulse leaves the terminal at 20 V (the high side stays on). From cc 1230, the terminal starts to float.
- **Without the motor**, the same 14 captures (prefix `tn`) give the same terminal levels within 0.2 V: 11.6 V median at cc 1232–1235, 12.1–12.2 V at cc 1240, 17.2 V at cc 1250, and the same limits at the low end (cc 19, 20, 25). So the motor does not set the floating level. The level comes from the TMC6200 and the bridge: the high side acts as a weak source follower, and the BM pin and MOSFET leakage pull the node down.

### The feature at about 60 counts from each end

Sample 0 at cc 52–65 (−0.4 A) and sample 1 at cc 1185–1198 (+0.5 A), both with a high sigma (0.2–0.35 A). Figure: `notes/scope/soa-edge-zoom.png`.

- **It is the switching ringing at the terminal edge, sampled by the ADC.** On the scope, SOA rings at about 30 MHz for about 0.1 µs after each terminal edge. The spread of single edges is 1–2.4 A from 0 to 0.09 µs, and less than 0.2 A from 0.15 µs.
- The evidence:
  - The sigma is high only at these cc values. The ringing is fast, so a few ns of jitter between the edge and the hold gives a large scatter.
  - At cc 1210–1229, sample 1 has a sigma of only 10–20 mA. So there, the hold is before the terminal edge, not 0.03–0.16 µs after it as the τ formula says.
  - The sign is opposite for the rising and falling edges. This agrees with a linear coupling of the edge (dV/dt).
  - It is the same with and without the motor.
  - At cc 0–20 (no high-side turn-on), sample 0 has a ±0.2 A pattern with a period of 5 counts (33 ns). That agrees with the 30 MHz ringing.
- **Correction to τ:** the terminal edge comes about 0.1 µs later than PWM edge + BBM. The TMC6200 datasheet gives the effective BBM as the gate slope time plus tBBM. The minimum pulse of 293 ns (= 168 ns + 125 ns) agrees with this. So τ = (formula above) − about 0.1 µs. The per-cc table and the sensitivity analysis use cc directly, so they do not change.

### Model for the implementation (decision: cc 25–1225, zero at 50 %)

1. **Correction:** `i_kj = (y_kj − o_k − E_kj[cc_edge]) / (G_k·R)`. cc_edge is the cc of the previous period for sample 0 and the cc of this period for sample 1.
2. **Table E_kj[cc]:** one value for each cc (1201 × 6 × int16 = 14 kB of RAM). There is no interpolation, because the duty is already quantized to cc. The table comes from `id_rec` with the rotor stationary.
3. **Sample selection by the timing sensitivity.** Under load, the terminal edge comes up to one BBM (25 counts) earlier than at zero current. The sensitivity is the largest change of E in that window (`rec_cc`, 9-point median, window clipped to cc 22–1228, run step at cc 1050 removed). Ranges where the sensitivity is ≤ 50 mA:

   | | Sample 0 (counter 0) | Sample 1 (TOP) | No sample ≤ 50 mA |
   |---|---|---|---|
   | A | cc 449–1225 | cc 100–323, 408–514 | cc 34–95 (≤ 100 mA from cc 73), cc 324–407 (up to 200 mA) |
   | B | cc 442–1225 | cc 71–740 | cc 31–70 (up to 114 mA) |
   | C | cc 433–1225 | cc 57–734 | cc 27–56 (up to 122 mA) |

   - Store the selection as a per-cc flag (s0, s1 or both). Where both are valid, use the inverse-variance mean. For A, sample 1 has a sigma of 60–130 mA and sample 0 has 12 mA.
   - cc 25–70: sample 0 is in the pre-response or spike range. In sample 1, the low tail grows with the pulse width. Under load, the pulse width changes by up to one BBM, so the error is up to about 100 mA.
   - A, cc 324–407: sample 0 is not settled yet, and sample 1 has the low-state ripple of A (230 kHz, probably from the LT3990), whose mean changes from run to run.
   - **Simplified layout** (figure `notes/scope/cal-table.png`): use one contiguous range for each sample, not a per-cc flag. The isolated "good" points (the bottom of the rising dip at cc ≈ 170, the plateau of sample 1 at cc 1140–1155) are flat only by chance, inside a transient, so do not use them.

     | | Sample 1 (TOP) | Sample 0 (counter 0) | Both: weight of sample 0 | Sensitivity > 50 mA |
     |---|---|---|---|---|
     | A | cc 25–514 | cc 449–1225 | 0.94 (σ 12 / 51 mA) | cc 31–95, 324–407 (max 201 mA), some single points |
     | B | cc 25–740 | cc 442–1225 | 0.74 (σ 19 / 31 mA) | cc 31–70 (max 114 mA) |
     | C | cc 25–734 | cc 433–1225 | 0.66 (σ 20 / 27 mA) | cc 27–56 (max 122 mA) |

     Data in the firmware: `int16_t E[3][2][1201]` in mA (14.4 kB), plus `x0[3]`, `x1[3]` and `w0[3]`. Median sensitivity: A 18 mA, B 13 mA, C 14 mA.

   - **Interpolated table** (figure `notes/scope/cal-table-32.png`): knots every 32 cc on one grid for all samples, from cc 25 to cc 1241 (39 knots). The knot values are a least-squares fit of a piecewise-linear function to the per-cc data (step at cc 1050 removed), only over the range of each sample. Data: `int16_t E[3][2][39]` in mA (468 bytes).

     Interpolation in integers: `d = cc − 25`, `i = d >> 5`, `f = d & 31`, `E = E[i] + (((E[i+1] − E[i])·f) >> 5)`. At cc 1225, i = 37, so i+1 = 38 is always in the table.

     Error against the 9-point smoothed per-cc data (max / rms, mA):

     | Spacing | Knots | Bytes | A0 | A1 | B0 | B1 | C0 | C1 |
     |---|---|---|---|---|---|---|---|---|
     | 16 cc | 76 | 912 | 13 / 4 | 51 / 12 | 9 / 3 | 10 / 3 | 12 / 3 | 11 / 3 |
     | 32 cc | 39 | 468 | 23 / 5 | 73 / 20 | 11 / 3 | 15 / 4 | 12 / 4 | 24 / 4 |

     The A1 error comes from cc 324–407, where the A low-state ripple causes changes from run to run. These cc values are outside the "≤ 50 mA" range for all spacings. In all other ranges, the 32-cc error is smaller than the run-to-run noise (σ 12–51 mA) and the timing sensitivity (median 13–18 mA).
4. **Supply voltage:** measure the table at the operating VBUS (one table for 15–20 V. Below that, the table changes).
5. **Open: the slow component.** Between run 1 and run 2 of `rec_cc`, sample 0 changed by 0.14–0.22 A (run 1 had switched for 90 s, then a short stop). This is larger than the target accuracy. Its time constant and cause (temperature or a slow electrical state) must be measured before the table is used in the loop.

   Figure `notes/scope/slow-component.png`:
   - (a) At cc 1050, sample 0 steps up by A0 149, B0 192 and C0 222 mA. The bridge was off for 13 s between the two runs.
   - (b) Sample 0 minus the reference sweep (`rec_motor`, an 11 s sweep in a different session) against time. After the 13 s stop, the difference decreases from +200 mA to about +50 mA in 5 s: a fast part in the first second, then a slower part. In run 1, the difference increases slowly, by about 80 mA in 50 s. One sweep cannot show if this is an effect of time or of cc.
   - (c) In each 50 ms run, sample 0 decreases from bin 1 to bin 8 by 38 mA at cc 450 and by 8 mA at cc 1220. This change has no step at cc 1050. Thus it is a separate, fast effect that each run starts again (the bridge is off for about 40 ms between runs).

   **Hold test at cc 625** (figure `notes/scope/hold-625.png`). The bridge was off for a long time before the test. Then `id_rec 625 625 1 20000` (one 1 s run) was repeated for 180 s, with a gap of about 0.1 s between the runs. Then the driver was off for 12 s (`drv_dis`), and the runs continued for 60 s. Each run now also prints the baseline (`rec ... base`, relative to ZERO_CAL). All values are relative to ZERO_CAL (bridge off):

   - **The high state is stable.** Sample 0 (terminal high, counter 0) changes by only −30 to −44 mA in 250 s.
   - **The low state drifts.** The baseline (all low sides on, no switching) increases by 663 (A), 843 (B) and 983 (C) mA in 180 s. Sample 1 (terminal low, TOP) increases by 615, 703 and 820 mA. The first run after the long rest has a baseline of −16 to −72 mA, that is, almost ZERO_CAL.
   - **Time constants** (fit of the baseline with 2 exponentials): τ 2.7–3.4 s and τ 36–39 s. The rms residual is 12–13 mA.
   - **Recovery when off.** After 12 s with the driver off, the baseline is 52–54 % back to the rested value. Then it increases again with the same shape.
   - **In each 1 s run**, bin 1 − bin 8 of sample 0 is only 0–4 mA. Thus the fast change in the 50 ms runs (8–38 mA, figure `slow-component.png` (c)) occurs in the first 0.1 s.

   Consequences:
   - The step at cc 1050 in `rec_cc` and the slow trend in run 1 are this drift. E = sample − baseline, and the baseline changed with the switching history. Thus the per-run E table contains the baseline state at the time of each run.
   - Use the reading relative to ZERO_CAL for the high-state samples. It does not need a baseline.
   - The low-state level (and with it the zero of the current) changes by up to about 1 A with the switching history. A fixed offset from ZERO_CAL is not sufficient for the low-state samples. The loop must track it, or must use the high-state samples. The drift is similar in A, B and C (within about ±20 %). Thus the condition i_A + i_B + i_C = 0 can find most of it.
   - The cause is not known. The time constants (3 s and 40 s) agree with a thermal effect, for example the TMC6200 die and package that the gate drive heats. A test: measure the TMC6200 case temperature during the hold test, or change the PWM frequency.

### Calibration table after a 10 s warmup (32 cc knots, with interval error)

Test: `drv_en`, then 10 runs of 1 s at cc 625 (warmup), then at once `id_rec 25 1225 1 2000` (1201 runs, 176 s). Log `cal.log`. Data `notes/scope/cal32.npz`. Figures `notes/scope/cal32-table.png` and `notes/scope/cal32-curves.png`.

- **Reference of the values.** Sample 0 (counter 0, terminal high) is relative to ZERO_CAL (bridge off), because the hold test shows that the high state does not drift. Sample 1 (TOP, terminal low) is relative to the static low level of the same run (the baseline), because the low state drifts.
- **Knots** at cc 25 + 32·i, i = 0..38 (cc 25 to 1241). The values are a least-squares fit of a piecewise-linear function to all per-cc points from cc 25 to 1225. The knot at cc 1241 is outside the data. It sets only the slope of the last interval.
- **err** = the max |measurement − interpolation| over the cc values from this knot to the next knot. The per-cc noise (median standard error of one point) is 1 mA for sample 0 and 2–6 mA for sample 1.
- **Large errors** (> 100 mA) occur only where the response changes fast: sample 0 at cc 25–120 (the ringing at about 60 counts) and sample 1 at cc 950–1210 (the same feature and the dip near the high end). In these ranges, one 32 cc interval cannot follow the curve. In the ranges that the sample selection uses (sample 0 from cc ≈ 440, sample 1 to cc ≈ 740), err is 1–22 mA, except A1 (25–107 mA, the A low-state ripple).
- **The baseline during the sweep** (relative to ZERO_CAL) increases from about 380/500/540 mA (A/B/C) at cc 25–125 to 530/720/780 mA at cc 425–525. Then it decreases to 250/365/405 mA at cc 1225. Thus the low-state drift does not depend only on the switching time. It decreases when the duty is high, that is, when the low side is on for a shorter time. A model of the drift must include the duty (or the low-side on time), not only the time.

Values in mA (1 mA = 0.1 mV at the ADC):

| cc | A0 | err | A1 | err | B0 | err | B1 | err | C0 | err | C1 | err |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 25 | 435 | 184 | 132 | 43 | 532 | 235 | 112 | 23 | 511 | 381 | 138 | 34 |
| 57 | 326 | 317 | 307 | 37 | 433 | 1142 | 242 | 16 | 605 | 914 | 232 | 21 |
| 89 | 639 | 125 | 422 | 33 | 861 | 352 | 257 | 16 | 887 | 356 | 224 | 24 |
| 121 | -303 | 44 | 440 | 40 | -71 | 57 | 262 | 14 | -33 | 42 | 233 | 19 |
| 153 | -1652 | 69 | 472 | 90 | -1083 | 65 | 262 | 21 | -1142 | 61 | 234 | 17 |
| 185 | -2272 | 131 | 445 | 57 | -1368 | 63 | 268 | 26 | -1535 | 59 | 238 | 19 |
| 217 | -2041 | 43 | 465 | 24 | -831 | 26 | 276 | 10 | -1103 | 26 | 244 | 16 |
| 249 | -1326 | 14 | 454 | 26 | 93 | 11 | 280 | 18 | -281 | 8 | 257 | 16 |
| 281 | -519 | 15 | 381 | 47 | 1070 | 16 | 293 | 19 | 622 | 16 | 254 | 20 |
| 313 | 246 | 24 | 355 | 107 | 1911 | 23 | 297 | 20 | 1417 | 19 | 260 | 16 |
| 345 | 834 | 16 | 225 | 76 | 2523 | 25 | 312 | 19 | 2007 | 25 | 268 | 19 |
| 377 | 1213 | 12 | 233 | 65 | 2910 | 11 | 329 | 17 | 2385 | 10 | 280 | 14 |
| 409 | 1464 | 10 | 292 | 30 | 3169 | 12 | 335 | 23 | 2646 | 12 | 282 | 20 |
| 441 | 1596 | 6 | 244 | 25 | 3299 | 5 | 355 | 16 | 2785 | 6 | 290 | 15 |
| 473 | 1651 | 3 | 298 | 40 | 3355 | 3 | 371 | 21 | 2850 | 3 | 297 | 22 |
| 505 | 1672 | 4 | 239 | 71 | 3369 | 3 | 403 | 24 | 2872 | 4 | 317 | 21 |
| 537 | 1668 | 2 | 368 | 39 | 3367 | 3 | 424 | 21 | 2876 | 3 | 331 | 17 |
| 569 | 1664 | 2 | 547 | 29 | 3364 | 10 | 448 | 21 | 2877 | 9 | 351 | 12 |
| 601 | 1658 | 1 | 686 | 48 | 3347 | 7 | 462 | 28 | 2861 | 7 | 377 | 20 |
| 633 | 1656 | 2 | 903 | 49 | 3337 | 3 | 499 | 18 | 2851 | 3 | 414 | 18 |
| 665 | 1656 | 3 | 1182 | 69 | 3339 | 19 | 555 | 22 | 2852 | 22 | 457 | 20 |
| 697 | 1666 | 1 | 1265 | 57 | 3396 | 6 | 578 | 23 | 2913 | 7 | 470 | 25 |
| 729 | 1672 | 2 | 1358 | 58 | 3397 | 3 | 554 | 27 | 2912 | 3 | 431 | 20 |
| 761 | 1678 | 2 | 1435 | 91 | 3404 | 2 | 405 | 35 | 2919 | 2 | 280 | 31 |
| 793 | 1687 | 1 | 974 | 103 | 3413 | 2 | 80 | 34 | 2927 | 2 | -40 | 32 |
| 825 | 1696 | 2 | 251 | 70 | 3422 | 3 | -524 | 44 | 2936 | 3 | -659 | 46 |
| 857 | 1703 | 2 | -871 | 112 | 3430 | 4 | -1537 | 35 | 2944 | 3 | -1669 | 44 |
| 889 | 1710 | 1 | -2764 | 72 | 3438 | 3 | -2918 | 30 | 2950 | 4 | -3070 | 29 |
| 921 | 1718 | 2 | -4824 | 95 | 3447 | 4 | -4583 | 42 | 2959 | 3 | -4784 | 38 |
| 953 | 1725 | 3 | -7180 | 99 | 3454 | 3 | -6237 | 123 | 2967 | 3 | -6510 | 120 |
| 985 | 1729 | 4 | -8725 | 231 | 3458 | 3 | -7273 | 171 | 2970 | 2 | -7677 | 187 |
| 1017 | 1738 | 3 | -8763 | 167 | 3466 | 2 | -6713 | 205 | 2978 | 3 | -7294 | 222 |
| 1049 | 1744 | 2 | -5939 | 177 | 3472 | 2 | -3665 | 176 | 2984 | 2 | -4381 | 207 |
| 1081 | 1752 | 2 | -1060 | 268 | 3479 | 3 | 1018 | 238 | 2991 | 3 | 315 | 235 |
| 1113 | 1760 | 4 | 1703 | 261 | 3487 | 5 | 3246 | 291 | 2999 | 4 | 2728 | 317 |
| 1145 | 1770 | 1 | 1447 | 294 | 3497 | 2 | 2817 | 930 | 3009 | 3 | 2336 | 903 |
| 1177 | 1778 | 4 | 1673 | 632 | 3505 | 3 | 3667 | 1034 | 3017 | 3 | 3136 | 1112 |
| 1209 | 1782 | 2 | 1722 | 139 | 3508 | 2 | 3285 | 110 | 3020 | 1 | 2728 | 83 |
| 1241 | 1787 | - | 1264 | - | 3515 | - | 2950 | - | 3027 | - | 2498 | - |

### Calibration sweep in one run (`id_cal`), mean of up and down

The firmware has a new test, `id_cal [lo_cc] [hi_cc] [loops_per_cc] [warm_s] [warm_cc]` (defaults 25 1225 100 10 625). It is one run without a stop:

1. Warmup: warm_s seconds of switching at warm_cc.
2. Start baseline: 200 loops at duty 0 (the second half is used).
3. Up sweep: cc lo..hi, loops_per_cc loops at each cc.
4. Down sweep: cc hi..lo, the same.
5. End baseline: 200 loops at duty 0.

After each cc change, the first 3 loops are not used (output delay). The values are relative to ZERO_CAL. Core 0 prints one line per cc with the up and the down value after the run (about 90 lines per second, thus 13 s for 1201 lines). With the defaults, the run takes 22 s (the earlier sweep took 176 s).

Test: 2 minutes with the bridge off, then `drv_en` and `id_cal` with the defaults. Log `cal3.log`. Data `notes/scope/calud.npz`. Figures `notes/scope/calud-table.png` and `notes/scope/calud-curves.png`.

- **Table values:** the mean of the up and the down value at each cc. This cancels a drift that is linear in time, because the mean time of the two values is the same for all cc. Sample 0 is relative to ZERO_CAL. Sample 1 is relative to the mean of the start and end baselines. The knots and "err" are the same as in the earlier section.
- **Baselines** (A/B/C, sample 0): start 455/603/629 mA, end 553/701/773 mA. Thus the low-state level increased by 98–144 mA in the 12 s of the sweep.
- **Up − down, sample 0, cc > 300:** median −4 mA. The high state does not drift, which agrees with the hold test.
- **Up − down, sample 1 and sample 0 below cc 300:** from −150 mA at cc 25 to +75 mA at cc 750 and −120 mA at cc 1000. A drift that is linear in time gives a straight line from about −100 to 0 mA. The measured shape is not straight. Thus the drift also depends on the duty, and the mean of up and down does not cancel all of it. The remaining error is probably some tens of mA in sample 1. This is not measured.
- **Comparison with the earlier sweep (separate runs, 2000 loops):** sample 0 above cc 440 agrees within 30 mA. The err of B1 and C1 in the used range is now 5–14 mA (before 14–28 mA), because the noise is less and the drift is cancelled. The large errors are at the same cc as before (the ringing near the ends and the deep dip of sample 1).

Values in mA (1 mA = 0.1 mV at the ADC):

| cc | A0 | err | A1 | err | B0 | err | B1 | err | C0 | err | C1 | err |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 25 | 617 | 186 | 118 | 22 | 728 | 326 | 93 | 12 | 733 | 385 | 109 | 10 |
| 57 | 451 | 402 | 287 | 13 | 645 | 1352 | 219 | 6 | 803 | 1139 | 200 | 7 |
| 89 | 735 | 124 | 410 | 14 | 1023 | 317 | 242 | 5 | 1059 | 336 | 199 | 5 |
| 121 | -222 | 42 | 432 | 15 | -100 | 51 | 249 | 7 | -71 | 47 | 214 | 8 |
| 153 | -1598 | 78 | 471 | 51 | -1092 | 56 | 250 | 9 | -1149 | 58 | 216 | 9 |
| 185 | -2245 | 84 | 453 | 79 | -1380 | 68 | 258 | 8 | -1564 | 71 | 220 | 7 |
| 217 | -2026 | 50 | 463 | 17 | -842 | 31 | 262 | 7 | -1138 | 28 | 223 | 6 |
| 249 | -1312 | 14 | 443 | 20 | 93 | 10 | 273 | 9 | -304 | 8 | 230 | 7 |
| 281 | -503 | 16 | 377 | 42 | 1070 | 16 | 278 | 7 | 605 | 13 | 237 | 5 |
| 313 | 269 | 25 | 344 | 102 | 1903 | 20 | 287 | 13 | 1396 | 18 | 240 | 10 |
| 345 | 859 | 19 | 200 | 75 | 2523 | 19 | 302 | 9 | 1995 | 17 | 247 | 9 |
| 377 | 1242 | 13 | 214 | 59 | 2930 | 14 | 318 | 6 | 2399 | 13 | 253 | 8 |
| 409 | 1492 | 10 | 275 | 28 | 3175 | 9 | 321 | 8 | 2647 | 11 | 255 | 7 |
| 441 | 1622 | 6 | 212 | 18 | 3302 | 5 | 331 | 7 | 2783 | 6 | 259 | 6 |
| 473 | 1677 | 3 | 259 | 31 | 3354 | 4 | 346 | 7 | 2844 | 4 | 266 | 6 |
| 505 | 1698 | 4 | 180 | 55 | 3366 | 3 | 376 | 9 | 2865 | 4 | 276 | 5 |
| 537 | 1692 | 2 | 324 | 33 | 3363 | 2 | 392 | 9 | 2868 | 3 | 290 | 8 |
| 569 | 1689 | 2 | 493 | 27 | 3357 | 2 | 411 | 8 | 2866 | 2 | 305 | 8 |
| 601 | 1684 | 2 | 628 | 26 | 3353 | 2 | 425 | 7 | 2863 | 2 | 333 | 10 |
| 633 | 1683 | 3 | 868 | 48 | 3353 | 3 | 451 | 8 | 2864 | 2 | 361 | 7 |
| 665 | 1683 | 2 | 1139 | 61 | 3354 | 3 | 504 | 8 | 2864 | 3 | 396 | 8 |
| 697 | 1688 | 1 | 1183 | 31 | 3358 | 2 | 517 | 11 | 2868 | 4 | 407 | 11 |
| 729 | 1695 | 2 | 1331 | 98 | 3365 | 2 | 491 | 14 | 2874 | 3 | 369 | 10 |
| 761 | 1701 | 2 | 1382 | 108 | 3371 | 2 | 347 | 24 | 2879 | 3 | 213 | 22 |
| 793 | 1709 | 1 | 952 | 68 | 3380 | 2 | 36 | 24 | 2888 | 2 | -96 | 30 |
| 825 | 1717 | 1 | 248 | 79 | 3389 | 2 | -557 | 33 | 2897 | 4 | -694 | 36 |
| 857 | 1726 | 1 | -846 | 113 | 3397 | 3 | -1549 | 29 | 2904 | 3 | -1690 | 37 |
| 889 | 1733 | 2 | -2732 | 86 | 3404 | 3 | -2910 | 22 | 2911 | 4 | -3065 | 25 |
| 921 | 1739 | 2 | -4781 | 89 | 3412 | 3 | -4571 | 41 | 2918 | 3 | -4765 | 39 |
| 953 | 1745 | 3 | -7148 | 106 | 3418 | 3 | -6242 | 108 | 2925 | 3 | -6514 | 103 |
| 985 | 1749 | 2 | -8760 | 230 | 3422 | 3 | -7337 | 172 | 2929 | 2 | -7736 | 175 |
| 1017 | 1758 | 2 | -8836 | 283 | 3431 | 2 | -6884 | 205 | 2937 | 1 | -7464 | 210 |
| 1049 | 1763 | 2 | -6176 | 185 | 3436 | 2 | -3962 | 191 | 2941 | 2 | -4690 | 167 |
| 1081 | 1771 | 3 | -1314 | 263 | 3444 | 3 | 689 | 234 | 2948 | 3 | -29 | 233 |
| 1113 | 1778 | 4 | 1541 | 276 | 3451 | 3 | 3018 | 241 | 2956 | 4 | 2475 | 246 |
| 1145 | 1786 | 2 | 1289 | 312 | 3459 | 2 | 2613 | 934 | 2964 | 2 | 2119 | 926 |
| 1177 | 1794 | 2 | 1438 | 675 | 3467 | 2 | 3323 | 1136 | 2972 | 2 | 2772 | 1103 |
| 1209 | 1797 | 1 | 1512 | 162 | 3470 | 2 | 3027 | 151 | 2975 | 2 | 2424 | 99 |
| 1241 | 1799 | - | 970 | - | 3471 | - | 2510 | - | 2978 | - | 2064 | - |

### Calibration table with 16 cc knots and the per-loop standard deviation

`id_cal` now also adds the squares of the samples. Each output line has a third group, `sd`: the standard deviation of one loop at this cc (the variance of the up part and of the down part, each about its own mean, pooled).

Test: `id_cal` with the defaults, after a long time with the bridge off. Log `cal4.log`. Data `notes/scope/cal16.npz`. Figures `notes/scope/cal16-table.png` and `notes/scope/cal16-curves.png`.

- **Knots** at cc 25 + 16·i, i = 0..75 (cc 25 to 1225, 76 knots). All knots are inside the data. For the firmware: `idx = (cc − 25) >> 4`, `frac = (cc − 25) & 15`. 76 knots × 6 samples × int16 = 912 bytes.
- **err**: the max |measurement − interpolation| from this knot to the next knot (the same as before).
- **sd**: the max standard deviation of one loop in the same interval. This is the random error of one sample in the loop. The standard error of the table value itself is sd / √194 (median 1–6 mA).
- **Baselines** (A/B/C, sample 0): start 407/622/630 mA, end 518/739/755 mA.

Results:

| Range | err | sd |
|---|---|---|
| Sample 0, cc ≥ 440 | 1–3 mA | 12–18 mA |
| B1, C1, cc ≤ 740 | 4–9 mA | 27–37 mA |
| A1, cc ≤ 510 | 6–80 mA | 46–126 mA (A low-state ripple) |
| Sample 0, cc 41–105 (ringing) | up to 986 mA | up to 3851 mA |
| Sample 1, cc 1145–1209 (ringing) | up to 877 mA | up to 3771 mA |

- The sd is large where E changes fast with cc: at the ringing near both ends, and on the slopes of the deep dip of sample 1 (cc 800–1100, 100–270 mA). In these ranges, the edge time changes a little from loop to loop, and the sample moves on a steep part of the response. Thus the sd is a direct measurement of the timing sensitivity.
- In the ranges that the sample selection uses, the random error of one sample (sd) is larger than the interpolation error (err). A total error estimate for one sample is err + 2·sd.

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

## 5. Open items

- The cause of the low-state drift (τ 3 s and 40 s, up to about 1 A). Measured in the hold test, see section 4, item 5.
- The scale difference of 9–13 % between the scope and the ADC.
- The terminal voltage at duties of 1 % and 99 % (scope on the terminal).
- The cause of the A noise in the low state (230 kHz).
- The response to the edges of other phases. The user said to ignore it, because its effect looks much smaller.
