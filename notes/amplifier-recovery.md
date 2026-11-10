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

## 5. Open items

- The time constant of the slow term (switching high level against DC high level). Run `id_rec 1000 1000 10 20000` after a run at 50 % and look at the 9 bins.
- The scale difference of 9–13 % between the scope and the ADC.
- The terminal voltage at duties of 1 % and 99 % (scope on the terminal).
- The cause of the A noise in the low state (230 kHz).
- The response to the edges of other phases. The user said to ignore it, because its effect looks much smaller.
