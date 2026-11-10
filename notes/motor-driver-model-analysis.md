# Motor and Driver Model Analysis

Date: 2026-11-09. Repository state: `main` at `b52293d`, with the user's uncommitted changes as they are now.

This note analyzes the motor model, the driver model, the current-sense model and the estimator. It compares the documents with the firmware. It lists gaps in order of impact on control quality. This note does not change code or documents.

## 0. Sources

| Source | Content | Status |
|---|---|---|
| `doc/smol-servo-modeling/smol-servo-modeling.tex` | "Servo Control Model" v0.1 | Almost empty. Section "TODO" says "Everything." (line 173–175). The text stops in the middle of a sentence at line 197. |
| `doc/smol-servo-modeling/smol-servo-modeling.bib` | Bibliography | Not used. The `.tex` loads `n17-servo-design.bib` (line 21). Both files have the same entry keys. |
| `doc/n17-servo-design/n17-servo-design.tex` | Design document, 2642 lines | Main source for the model. It describes the older STM32G431 + DRV8323 board (line 180, 184). |
| `doc/livebook/driver-design.livemd` | Gate driver and MOSFET loss calculation | DRV8323 + FDMC86570L only. |
| `firmware_rpico/src/smol_servo_model.c` | Draft model and estimator | Not in the build (`firmware_rpico/CMakeLists.txt:73-89`). It does not compile. |
| `firmware_rpico/src/smol_servo_isense.c` | Draft current-sense observation | Not in the build. It does not compile. |
| `firmware_rpico/src/tmc6200.c`, `.h`, `smol_servo_drv.c` | TMC6200 SPI driver and driver control | In the build. `main.c:129` calls `tmc6200_spi_init`. `main.c:131` calls `smol_drv_init`. |
| `stm32-servo-firmware/src/ssf_kalman.c`, `.h` | Generic sequential Kalman filter | No callers. |
| `stm32-servo-firmware/src/ssf_mctrl_estimator.c` | 2-state position and speed Kalman filter | The estimator that the STM32 firmware used. Read for comparison. |

Supporting files that I also read: `firmware_rpico/src/smol_servo_parameters.h`, `smol_servo_pwm.c`, `smol_servo_adc.c`, `smol_servo_loop.c`, `smol_servo_mag.c`, `stm32-servo-firmware/src/ssf_spi_tmc6200.c`, and the value lists of the KiCad schematics in `hardware/smol-servo-nomcu-tmc6200/` and `hardware/smol-servo-rp-nodrv/`.

## Summary: top 5 findings

1. **No working model or estimator exists for the RP2350 firmware.** `smol_servo_model.c` and `smol_servo_isense.c` are sketches. They are not in the build and they do not compile. The integration math in `smol_servo_model.c:211-217` has errors. The only estimator that ran is the 2-state STM32 filter (`ssf_mctrl_estimator.c:219-353`). It does not use current or voltage.
2. **The torque and back-EMF equations use different angle functions.** The documents and the code comment use `E = k ω cos(θe)` and `τ = −k i sin(θe)` (`n17-servo-design.tex:975`, `:985`, `:1808`, `:1822`, `smol_servo_model.c:96-99`). For a lossless machine, `E·i` must equal `τ·ω`. These forms break that rule. A controller built on them puts the current 90° electrical from the torque axis.
3. **Sensor and actuation delays are large in electrical angle and no model includes them.** The AS5047D delay is about 100 µs (`n17-servo-design.tex:523`, `:532`). Both firmwares read `ANGLEUNC`, the angle without delay compensation (`smol_servo_mag.c:44-46`, `ssf_mctrl_estimator.c:382`). The PWM output has a 5-PWM-cycle (83 µs) delay (`smol_servo_loop.c:47-54`). With 50 pole pairs, 180 µs at 10 rev/s is 32° electrical.
4. **The current-sense model is incomplete.** The ADC samples only one phase per PWM period (`smol_servo_adc.c:16-22`). The three phase currents are thus never simultaneous. The zero offset, the gain chain and the high-side versus low-side offset (`n17-servo-design.tex:517`) have no calibrated model. The firmware now sets the gain at boot and measures the zero with the driver disabled (section 5.4).
5. **The TMC6200 driver code is now correct (update of 2026-11-10).** The first version of this note listed SPI and init defects. Commits `aa08e8a`, `7f20adf` and `3eeb961` (2026-11-09) correct all of them. The driver state is now known: `smol_drv_init` writes `GCONF` and `DRV_CONF` and reads them back (`smol_servo_drv.c:79-129`). Section 5.5 gives the details.

## 1. Motor model

### 1.1 Model forms in the documents

The design document has four model forms. They are not one consistent model.

**A. Single-phase R-L-EMF winding** (`n17-servo-design.tex:964-978`, Figure at `:921-933`):

```
U = E + R·i + L·di/dt
E_k = k_e · ω_m · cos(n·θ_m + φ0) = k_e · ω_m · cos(θe + φ0)
τ = −k · sin(θe + φ0) · i                          (line 985)
```

**B. QD form of the single-phase model** (`n17-servo-design.tex:989-1037`). The document projects the phase current into `i_d = I_d cos θ` and `i_q = −I_q sin θ`. It then gets:

```
U_q = R·I_q + L·ω_e·I_d + L·dI_q/dt
U_d = k_e·ω_e + R·I_d − L·ω_e·I_q + L·dI_d/dt
```

The back-EMF term is on the d axis. This follows from the cos form of `E`. In the usual PMSM convention the back-EMF is on the q axis. Line 975 uses `ω_m` and line 1035 uses `ω_e`, so the meaning of `k_e` changes by the factor `n`.

**C. DC motor state equation** (`n17-servo-design.tex:1785-1793`):

```
L·di/dt = U − R·i − k_e·ω
J·dω/dt = k_e·i − k_d·ω − Γ_d
dx/dt   = ω
```

The torque constant and the EMF constant are the same symbol here. That is correct in SI units.

**D. 2-phase stepper model** (`n17-servo-design.tex:1795-1832`):

```
S = 2·m·n         S = 200 steps, m = 2 phases → n = 50 pole pairs
τ_j = k_m · sin(n·θ_m + φ0j) · i_j                  (line 1808)
J·dω/dt = Σ τ_j − k_d·ω_m − Γ_d                     (line 1814)
Σ τ_j = k_m · I · sin φ_m                           (line 1815)
U_j = E_j + R·i_j + L·di_j/dt                       (line 1821)
E_j = k_e · ω_m · cos(n·θ_m + φ0j)                  (line 1822)
```

The sign of the torque term at line 1808 is the opposite of the sign at line 985.

**E. 3-phase star and 2-phase-on-3-terminal circuits** (`n17-servo-design.tex:1265-1271`, `:1308-1779`). These give the terminal-to-phase relations, resistance identification (`:1707-1751`) and the 2-phase drive waveforms (`:1753-1779`).

**F. Cogging model** (`n17-servo-design.tex:2200-2413`). The first-order cogging torque is `τ_cog = −k_cog sin(6θe)` (`:2294`). This is for a 3-phase motor. Near a stable point the document uses a linear mass-spring-damper model (`:2397-2412`).

### 1.2 State variables

| Variable | Document | `smol_servo_model.c` |
|---|---|---|
| Phase currents `i_j` | `:1821` | No. Only `i_d`, `i_q` (`:52-53`). |
| `i_d`, `i_q` | `:992`, `:998` | Yes, with scalar variances (`:52-58`). |
| Rotor position, speed | `:1791`, `:2106-2127` | Yes (`:61-64`). |
| Current-sense zero offsets | No | Yes, 3 values (`:44-46`). |
| Bus voltage | No | Yes (`:48-50`). |
| `R`, `L`, `k_e` as states | Proposed (`:1990-1992`) | Fields exist (`:40-42`) but have no variances and no update. |
| Rotor inertia | Parameter `J` | Used (`:211`) but not a field of the struct. |

### 1.3 Parameters

| Parameter | Value | Source | Notes |
|---|---|---|---|
| Motor | 17HS16-2004S-C2 | `n17-servo-design.tex:2524` | Bipolar, 1.8° step. |
| Holding torque | 0.45 N·m | `:2531` | No `k_t` value comes from it in the documents. |
| `R` | 1.1 Ω ±10 % (datasheet). 1.25–1.32 Ω at 15 kHz (LCR meter). | `:2533` | The LCR value at 15 kHz includes AC loss. |
| `L` | 2.6 mH ±20 % (datasheet). 1.46–1.52 mH at 15 kHz, 0.47–0.51 mH at 200 kHz, about 1.20 mH at 50 kHz (estimate). | `:2532` | Measured `L` is 40–55 % below the datasheet value. `L` drops with frequency. |
| Rated current | 2 A | `:2534` | |
| Rotor inertia `J` | 54 g·cm² | `:2535` | Rotor only. Load inertia is unknown. |
| Pole pairs `n` | 50 | `:1803` | |
| `k_e`, `k_d`, `k_s`, encoder offset, encoder delay | Unknown | Table at `:1857-1875` | The table has "???" for `k_s` and for the encoder delay. |
| BLDC 42BLF01 | 4 pole pairs, 1.8 Ω line-to-line, 0.54 mH, 4.4 V/kRPM | `:2543-2561` | Second test motor. |
| Electrical time constant | 2.6 mH / 1.1 Ω = 2.4 ms (datasheet). About 1.2 mH / 1.3 Ω = 0.9 ms (measured). | Calculated from `:2532-2533` | |
| Current slew at standstill | 9.2 kA/s at 24 V | `:2419-2431` | Uses the datasheet `L`. With measured `L` the slew is about 2× higher. |

### 1.4 Assumptions

Assumptions that the documents state:

- All phases are equal (`smol-servo-modeling.tex:187`, `n17-servo-design.tex:1825`).
- The magnetic characteristic is sinusoidal (`n17-servo-design.tex:982`, `:1805`).
- The motor is a 2-phase PM motor model (footnote at `:1811`).
- For estimation, `φ_Z ≈ 0` (`:1970`). This means `ωL ≪ R`. That is not true for this motor above a few electrical Hz: at 1 rev/s, `ω_e = 314 rad/s` and `ωL ≈ 0.4 Ω`.
- Constant speed and current for the voltage analysis (`:1058`).
- Viscous friction only (`:1900`).

Assumptions that the documents and code do not state, but that the equations imply:

- No magnetic saturation. `L` is a constant. No `L(i)` model exists. The measured `L` drop with frequency (`:2532`) shows eddy-current effects that the model does not include.
- No saliency. `L_d = L_q = L`. A hybrid stepper has a small saliency. This affects sensorless angle estimation and the reluctance torque.
- No mutual inductance between the two phases.
- No detent torque model for the 2-phase hybrid stepper. The cogging model at `:2294` uses `sin(6θe)`, which is the 3-phase case. The dominant detent harmonic of a 2-phase hybrid stepper is at 4× the electrical angle (one period per full step).
- No temperature dependence of `R` or of the magnet flux.
- No load model. `Γ_d` is a disturbance (`:1790`) but no estimator includes it.

### 1.5 Model in `smol_servo_model.c`

The file header lists the sensors, controls and hidden states (`smol_servo_model.c:3-33`). It assumes an R-L-BEMF series circuit per phase (`:6`). The prediction step (`:83-223`) has these parts:

- Time step from a 150 MHz clock count (`:84`).
- Rotor integration (`:207-213`):
  ```
  a = i_q · k / J
  v_new = v + h² · a           (line 212, should be h · a)
  x_new = h · v + 0.5 · h² · a (line 213, the old x is missing)
  ```
- Variance propagation (`:215-217`): `acc_p = i_q_p + k_p + J_p`. This adds the variances of the factors of a product. That is not correct. The variance of a product needs the means and the relative variances. The code keeps no cross-covariance between position and speed. Thus the code is not a Kalman filter. It is a set of separate scalar filters.
- Offset random walk: `zero_p += zero_s · dt` (`:221`). `dt` is not defined. `VSENSE_ZERO_V_UPDATE_VARIANCE` is (10 mV)² per second (`smol_servo_parameters.h:71-72`).
- The observation function (`:225-231`) has `?` as placeholders.

Compile errors: `_model` is an array but the code uses it as a struct (`:72-79`). The fields `vsense`, `vsense_p`, `vsense_s`, `rotor_inertia`, `rotor_inertia_p` and `rotor` do not exist. `i_d[vsense_index]` indexes a float (`:228`). The `#include "smol_servo"` has no `.h` (`:1`).

The Wye-Delta comment (`:106-193`) has a math error. It finds `V_1 = |V|/3 · cos(x − 60°)`. For balanced line voltages with `V_AB = |V| cos x`, the result is `V_1 = |V|/√3 · cos(x + 30°)` (or `−30°` with the other phase order). The magnitude is wrong by √3. The design document has the correct √3 factor for line differences (`n17-servo-design.tex:1289-1291`). This error affects only the 3-phase case.

The mechanical angle formula `q = 2·num_phases/(num_slots·num_poles)` and `mechanical = (electrical · q) % 2π` (`:203-205`) does not agree with the document (`n = S/(2m)`, `:1800-1803`). The modulo also removes full turns.

## 2. Driver and current-sense model

### 2.1 Bridge topology

- Three half bridges with discrete N-MOSFETs (`n17-servo-design.tex:180`, `smol-servo-modeling.tex:189`).
- Current board: TMC6200 gate driver (`hardware/smol-servo-nomcu-tmc6200/smol-servo-nomcu-tmc6200.kicad_sch`, 6 × "TMC6200" symbol units). MOSFET NVTFS5C658NL (7 parts: 6 bridge + 1 brake resistor switch, by count). Shunt "5m" (3 parts). These match `ISENSE_R = 5 mΩ` (`smol_servo_parameters.h:65`).
- A fourth PWM output drives a brake (dissipation) resistor (`smol_servo_pwm.c:6`, `:36-37`).
- A 2-phase motor connects to 3 terminals. The two phases share one half bridge (`n17-servo-design.tex:1308-1372`). The shared terminal carries the sum of both phase currents.
- 2-phase drive waveforms with a moving midpoint give `U_P = √2 · U_naive` (`:1757-1771`). The phase amplitude is `U_bus/√2`. The naive midpoint gives `U_bus/2`.
- The documents model each switch as a resistor `R_T` and the shunt as `R_S` (`:1415-1572`, `:1707-1751`). Firmware draft lists `RDS_on` as a hidden state (`smol_servo_model.c:23`).

### 2.2 PWM and timing

| Item | Design document | RP2350 firmware |
|---|---|---|
| PWM frequency | 50 kHz, up/down (`n17-servo-design.tex:407-409`) | 60 kHz, phase-correct (`smol_servo_parameters.h:46-47`, `smol_servo_pwm.c:31-32`, `:145-153`) |
| PWM resolution | Not stated | 1250 counts per half period, about 10.3 bits (`smol_servo_pwm.c:13-15`) |
| Control loop | 16.7 kHz (`:409`) | 20 kHz, one loop per 3 PWM periods (`smol_servo_parameters.h:41-44`) |
| ADC | STM32 ADC2, 16× oversampling (`:505`) | RP2350 ADC at 240 kHz, 2 µs per sample (`smol_servo_adc.c:8`, `smol_servo_parameters.h:49-52`) |
| Encoder | Once per control cycle (`:534`) | 80 kHz, 4 reads per loop (`smol_servo_parameters.h:54-56`) |
| Sensor-to-PWM delay | Not stated | 5 PWM periods = 83 µs (`smol_servo_loop.c:47-54`) |

The model in `smol_servo_model.c` uses `U_phase[n] = U_bus · PWM[n]` (`:104`). This assumes an ideal switch.

### 2.3 Dead time

- The documents do not model dead time.
- The TMC6200 sets break-before-make time with `DRV_CONF.BBMCLKS` (`tmc6200.h:98`). The datasheet gives 42 ns × BBMCLKS, with an OTP reset default of 1..4, thus 42–168 ns (`doc/references/TMC6200_datasheet_rev1.08.pdf`, DRV_CONF register table). The firmware writes `BBMCLKS = 4` (168 ns) at boot (`smol_servo_parameters.h:75`, `smol_servo_drv.c:115`). The config key `bbmclks` can change it (`smol_config.c:54`). `smol_drv_bbm_ns()` returns the value in ns (`smol_servo_drv.c:245`).
- The firmware uses PWM/EN pairs per phase (`smol_servo_pwm.c:4`, `:190-196`). This matches the TMC6200 single-line mode. `smol_drv_init` sets `GCONF.singleline = 1` (`smol_servo_drv.c:108`). In this mode the TMC6200 makes the dead time.
- The model has no voltage error term for dead time, diode drop or `RDS_on`. At standstill and low speed, this error is a large part of the small phase voltage.

### 2.4 Current-sense path

Signal chain, from the firmware:

```
i_phase → R_s = 5 mΩ → TMC6200 amplifier (gain G) → [scaling not documented] → RP2350 ADC (12 bit, 3.3 V)
v_adc = V0 + G·R_s·i   →   i = (v_adc − V0) / (G·R_s)
```

| Item | Value | Source |
|---|---|---|
| Shunt position | Load side (in the phase line) | `n17-servo-design.tex:509` |
| Gain options | {5, 10, 10, 20} | `smol_servo_parameters.h:67` |
| Nominal zero `V0` | 5/3 V. The comment says the TMC6200 uses its own 5 V supply, so the zero moves relative to the ADC supply. | `smol_servo_parameters.h:68-69` |
| Zero prior | σ = 100 mV | `smol_servo_parameters.h:70` |
| Zero drift | σ = 10 mV per √s, random walk | `smol_servo_parameters.h:71-72` |
| ADC noise | σ = 16 LSB = 12.9 mV | `smol_servo_isense.c:11` |
| Settling after a switch edge | 2–4 µs. Firmware uses 600 clocks = 4 µs. | `n17-servo-design.tex:511`, `smol_servo_isense.c:4`, `:7` |
| Half sample window | 150 clocks = 1 µs | `smol_servo_isense.c:9` |
| Sample spikes | About 140 mV drop during the sample | `n17-servo-design.tex:513` |
| Common-mode residue | High-side and low-side samples have different offsets | `n17-servo-design.tex:517` |
| ADC scale | `3.3 V / 4095` | `smol_servo_adc.c:422` |

Derived values assume no divider between the amplifier and the ADC. With G = 20 the scale is 100 mV/A and 1 LSB is 8 mA. The noise of 12.9 mV is then 129 mA, or 6.5 % of the 2 A rated current. With G = 5 the range is about ±64 A and the noise is 516 mA. The TMC6200 datasheet gives a zero of 5VOUT/3 when the reference pin is open (pin table, CURU/V/W and reference pin). This is the 5/3 V of the firmware, so no divider is necessary to explain the zero. Check the schematic for a divider.

Sample plan (`smol_servo_adc.c:16-22`): each PWM period samples one phase current twice (at counter 0 and at TOP). The other slots sample voltages and temperature. Thus each phase current has one sample point per 50 µs loop, and the three phases are 16.7 µs apart. `smol_isense_update_phase_current` picks the sample that is far from an edge (`smol_servo_isense.c:14-31`). The function stops at `float isense =` (`:29`) and does not compile. `VSENSE_VAR` uses the undefined `ISENSE_SIGMA` (`:12`).

The 2-phase connection gives a useful constraint. The sum of the three terminal currents is zero at all times. With three sensors and two independent currents, the offsets of the sensors are partly observable during operation.

### 2.5 Supply voltage

- Design range: 12–48 V nominal, 60 V peak (`n17-servo-design.tex:180`).
- USB PD: up to 48 V with EPR (`n17-servo-design.tex:2616`, commit `b52293d`).
- VBUS measurement: divider 120 kΩ / 5.1 kΩ, full scale about 81 V (`smol_servo_parameters.h:76-94`). The ADC input resistance (100 MΩ) is in the divider formula.
- Bus capacitor parts: "10u 60V" and "2.2uF 75V" in the schematic value list (`smol-servo-nomcu-tmc6200.kicad_sch`).
- A USB PD source cannot take energy back. Motor braking thus raises the bus voltage. The brake resistor and "flux braking" (`n17-servo-design.tex:172-173`, `:1010-1014`) are the planned remedies. No model of bus energy exists. At 48 V the margin to the 60 V parts is 12 V.

### 2.6 Loss model (`driver-design.livemd`)

- Static loss `R_DS(on) · I²` (line 71), switching loss `f · V_DS · I · (Q_GS2 + Q_GD)/I_G` (line 78), gate loss `f · V_G · Q_G(tot)` (line 93).
- Example: FDMC86570L with DRV8323, 48 V, 8 A, 50 kHz, 1 A gate current (lines 153–196). These equations give about 0.44 W static, 0.39 W switching and 0.063 W gate loss. The gate charge at 12 V comes from a linear extrapolation (lines 145–149).
- The body-diode reverse-recovery loss uses `V_SD` (line 109). The usual form uses the bus voltage. With 48 V instead of 1.3 V the value is 37× higher.
- The body-diode conduction loss uses `V_DS × I_S` (line 103). The usual form is `V_SD × I × (2 · t_dead · f)`.
- `t_rr` is in the data (line 184) but no function uses it.
- The current board uses NVTFS5C658NL and TMC6200, not FDMC86570L and DRV8323.

## 3. Estimator

### 3.1 Design document

- Linear discrete Kalman filter in standard form (`n17-servo-design.tex:2010-2056`). The document states that `Q` and `R` change with time (`:2043`). It recommends the Joseph form (`:2051-2056`).
- Sequential scalar updates (`:2059-2091`).
- Simplest model: state `[x, ω]`, `A = [[1, h], [0, 1]]`, `H = [1, 0]`, no input (`:2094-2150`).
- Noise model (`:2161-2173`, table `:2182-2197`):
  ```
  Q = diag((k_Q1·h)², (k_Q2·h)²)     k_Q1 = 0.2π, k_Q2 = 2π
  R = (k_R1 + k_R2·λ + k_R3·ω·λ)²    k_R1 = 0.0222 rad, k_R2 = 2π, k_R3 = 0
  ```
  `k_R1` combines the AS5047D noise (σ = 0.068°) and the INL (1.2°). `λ` is the time since the last observation.
- The document proposes motor parameters as Kalman states (`:1990-1992`). It gives no model for this.

### 3.2 STM32 operational estimator (`ssf_mctrl_estimator.c`)

- State `[x, ω]` (`:248-250`), constant `h = 60 µs` (`:226`). The line in a comment uses the real time step (`:225`).
- `Q` matches the document (`:234-239`). `R = 0.0222²` (`:283-284`). The time terms of `R` are not used.
- The filter moves the measurement forward to "now" with `z = x_enc + λ·ω_pre` (`:275-281`). It wraps the angle error into ±π (`:281`, `:342`).
- Covariance update `(I − KH)·P` (`:307-315`), not the Joseph form.
- The encoder value is `ANGLEUNC` (`:382`). The timestamp is the middle of the SPI transfer (`:375-376`). The 100 µs sensor delay is not in `λ`.

### 3.3 STM32 generic filter (`ssf_kalman.c`)

No code calls this file. It has these defects:

- `kalman_predict` adds `A·x` to `x` (`:154-155`). The result is `x + A·x`, not `A·x`.
- `kalman_predict` computes `A·P·Aᵀ` into `P_tmp` but does not add `Q` and does not copy the result to `P` (`:162-163`).
- `mtransformadd` increments `k` in the inner loop that tests `l` (`:130`). This loop does not stop. The size checks and the strides are also wrong for `A·B·Aᵀ` (`:114-136`).
- `kalman_measure` indexes the row of `H` with `H_dims.rows · i` (`:181`). It must use the number of columns (`x_dim`). It writes `K` with stride `K_dims.rows` (`:194`). It must use `z_dim`.
- The Joseph update calls `mmultransposeadd(P_tmp, P, IKH)` (`:216`). This gives `P·(I−KH)·Pᵀ`, not `(I−KH)·P·(I−KH)ᵀ`.
- The uncommitted change in `ssf_kalman.h:10` corrects only a comment.

### 3.4 RP2350 draft (`smol_servo_model.c`)

See section 1.5. The draft adds current-sense offsets and bus voltage to the state. This is a good direction. It uses separate scalar variances, so it cannot track correlations. The correlation between offset and current is the main thing that makes the offset observable.

## 4. Agreement and differences

| Topic | Agreement | Difference |
|---|---|---|
| Phase circuit | Docs and code use R-L-EMF per phase (`n17-servo-design.tex:968`, `smol_servo_model.c:6`, `:100-103`). | None. |
| EMF and torque angle | Both use `E ∝ cos`, `τ ∝ −sin` (`:975`, `:985`, `smol_servo_model.c:96-99`). | Both are wrong together (finding 2). Line 1808 uses `+sin`. |
| `ω_m` versus `ω_e` in `k_e` | | `:975` and `:1822` use `ω_m`. `:1027` and `:1035` use `ω_e`. |
| Pole pairs | | Doc: `n = S/(2m) = 50` (`:1800-1803`). Code: `q = 2·phases/(slots·poles)` (`smol_servo_model.c:203`). |
| Position filter | STM32 code and doc use the same `A`, `H`, `Q`, `k_R1` (`:2131-2190`, `ssf_mctrl_estimator.c:226-284`). | Doc `R` has time terms. Code does not use them. Doc recommends Joseph form. Code uses the short form. |
| Generic filter | `ssf_kalman.c` follows the sequential form of `:2079-2091`. | Implementation is broken (section 3.3). |
| Hardware | Doc says 3 discrete half bridges. Board agrees. | Doc: STM32G431, DRV8323, low-side shunts, 15 mΩ (`:184`, `:391`, `:503`). Board: RP2350, TMC6200, load-side shunts, 5 mΩ. |
| PWM, loop rate | | Doc: 50 kHz, 16.7 kHz. Firmware: 60 kHz, 20 kHz. |
| Current-sense settling | Doc 2–4 µs (`:511`). Firmware 4 µs (`smol_servo_isense.c:7`). | None. |
| MOSFET | | Livebook: FDMC86570L. Board: NVTFS5C658NL. |
| 3-phase line voltage | Doc has √3 (`:1289-1291`, `smol-servo-modeling.tex:195`). | Code comment has 1 (`smol_servo_model.c:182`). |
| Cogging | | Doc model is for 3 phases (`sin 6θe`, `:2294`). Target motor is a 2-phase stepper (4θe). |
| TMC6200 enable | STM32 code clears `disable` to enable (`ssf_spi_tmc6200.c:192`). | Agree. RP2350 code sets `disable = !en` (`tmc6200.c:127`). `smol_drv_enable` clears `disable` only after a PD contract of at least 9 V and a fault check (`smol_servo_drv.c:271`). |

## 5. Gaps, errors and open questions

The order is by impact on control quality. The highest impact is first.

### 5.1 No working estimator for the RP2350 firmware

- **Problem.** The model and current-sense files do not compile and are not in the build (section 0, 1.5). The integration in `smol_servo_model.c:212-213` uses `h²` for speed and drops the old position. The code keeps separate variances and adds them in the wrong way (`:215-217`).
- **Next step.** Write the state-space model in `smol-servo-modeling.tex` first: state `[θ, ω, i_α, i_β, offsets]`, input `[d_A, d_B, d_C, U_bus]`, and measurements. Then put a full-covariance filter (EKF) into `smol_servo_model.c`. Keep the STM32 2-state filter as a fallback for system identification. Test the filter offline against logged ADC and encoder data before the motor runs closed loop.

### 5.2 Torque and EMF angle functions do not agree

- **Problem.** `E ∝ cos(θe)` with `τ ∝ −sin(θe)` breaks power balance (finding 2). The QD equations at `n17-servo-design.tex:1034-1035` inherit the error. The torque sign is also different at `:985` and `:1808`.
- **Next step.** Define the flux linkage per phase as `ψ_j = ψ_m · cos(θe − φ_j)`. Then `E_j = −ψ_m · ω_e · sin(θe − φ_j)` and `τ_j = −n · ψ_m · i_j · sin(θe − φ_j)`. Thus `k_t = k_e = n·ψ_m` in SI units (per `ω_m`). Use the standard QD form: `U_d = R·i_d + L·di_d/dt − ω_e·L·i_q` and `U_q = R·i_q + L·di_q/dt + ω_e·L·i_d + ω_e·ψ_m`. Correct `:975-1037`, `:1808-1822` and `smol_servo_model.c:96-99`.

### 5.3 Encoder and PWM delays are not in the model

- **Problem.** Sensor delay is about 100 µs (`n17-servo-design.tex:523`). PWM output delay is 83 µs (`smol_servo_loop.c:54`). With 50 pole pairs, 180 µs is 3.2° electrical per rev/s, and 32° at 10 rev/s. Both firmwares read `ANGLEUNC`.
- **Next step.** Choose one method. Method A: read `ANGLECOM` (the AS5047D delay compensation) and measure the remaining delay. Method B: keep `ANGLEUNC` and put the sensor delay into the measurement time of the filter. In both methods, predict the rotor angle to the middle of the PWM period that applies the new duty. Measure the delay on the bench with a constant-speed run and the zero crossings of the EMF.

### 5.4 Current-sense model and calibration

- **Problem.** The ADC samples the phases at different times (`smol_servo_adc.c:16-22`). The gain chain from amplifier to ADC is not documented (`smol_servo_parameters.h:68-69`). The high-side and low-side samples have different offsets (`n17-servo-design.tex:517`). The firmware now writes the TMC6200 gain bits at boot (gain 20, `smol_servo_parameters.h:72`). The offset drift value (10 mV/√s) has no measurement.
- **Next step.**
  1. Document the analog path from TMC6200 output to ADC pin in the note or the modeling document. Check the schematic for a divider. The datasheet zero is 5VOUT/3.
  2. Done. `smol_drv_init` sets `GCONF.amplification_x5` and `amplification_x1` from `ISENSE_AMPLIFICATION_CODE` and reads them back (`smol_servo_drv.c:110-111`).
  3. Done. The loop state `SMOL_LOOP_ZERO_CAL` measures the zero of each channel with the TMC6200 disabled. It measures the counter-0 and the TOP sample positions separately. It checks the mean and the standard deviation (`smol_servo_loop.c:380-420`). Still open: a schedule for this calibration at each power-up, as the datasheet recommends.
  4. In the filter, use each phase sample at its own time with the model, not as a simultaneous set. Use the zero-sum constraint of the terminal currents as a measurement of the offsets.
  5. Log the zero over 10 minutes and over temperature to get the real drift.

### 5.5 TMC6200 driver code

- **Status (update of 2026-11-10).** The first version of this note listed these defects. Commits `aa08e8a`, `7f20adf` and `3eeb961` (2026-11-09) correct all of them:
  - The byte count of `spi_write_read_blocking` counted as an error. Now `_tmc6200_transfer` writes the SPI FIFO directly and returns no count (`tmc6200.c:37-63`).
  - The code did not drive NSS during a transfer. Now the SPI peripheral drives CSn in mode 3. The code puts all 5 bytes into the TX FIFO with interrupts disabled, so CSn stays low for the full 40-bit datagram (`tmc6200.c:49-52`, `:159`).
  - The write sent only the low byte. Now the code packs the bytes MSB first with `>>` (`tmc6200.c:79-85`).
  - `tmc6200_enable_driver(true)` set `disable = 1`. Now it sets `disable = !en` (`tmc6200.c:127`).
  - `tmc6200_spi_init` had no `return`. Now it returns 0 (`tmc6200.c:161`).
  - The firmware did not write `GCONF` or `DRV_CONF`. Now `smol_drv_init` checks the version, writes `GCONF` (singleline, gain 20, `disable = 1`) and `DRV_CONF` (BBMCLKS 4, DRVSTRENGTH 2), and reads them back (`smol_servo_drv.c:79-129`).
- **Remaining items.**
  - The same byte-packing defect is still in the old STM32 code (`ssf_spi_tmc6200.c:107-113`). That code is not in use.
  - The datasheet says that the TMC6200 returns the read data in the same read access (SPI section). The code reads the data from the correct datagram.

### 5.6 Dead time and switch voltage error

- **Problem.** The model uses `U = U_bus · d` (`smol_servo_model.c:104`). The firmware sets the BBM time to 168 ns, but the model does not use it (section 2.3). At standstill a stepper servo uses small voltages. For example, 2 A × 1.1 Ω = 2.2 V, which is 4.6 % of 48 V. A dead-time error of 100–200 ns at 60 kHz is 0.6–1.2 % of the period, or 0.3–0.6 V at 48 V.
- **Next step.** Use `smol_drv_bbm_ns()` as `t_dead`. Add `ΔU = −sign(i) · (t_dead · f_pwm · U_bus + U_diode)` and `R_T · i` to the voltage model. Identify `R_T` with the resistance method at `n17-servo-design.tex:1707-1751`.

### 5.7 Inductance, saturation, detent torque

- **Problem.** Measured `L` is 1.2–1.5 mH, not 2.6 mH, and depends on frequency (`n17-servo-design.tex:2532`). No model for `L(i)`, saliency or detent torque of a 2-phase hybrid stepper exists. The cogging model is for 3 phases (`:2294`).
- **Next step.** Use the impedance method (`:743-787`) at the PWM frequency and at the current-loop bandwidth. Measure at 3 or more DC bias currents to see saturation. Measure the detent torque against angle at zero current with the encoder. Fit harmonics at 4θe and 8θe. Put the result as a feed-forward table.

### 5.8 Mechanical model and load

- **Problem.** The documents give only the rotor inertia (54 g·cm², `:2535`). `k_d`, `k_s` and the load are unknown (`:1857-1875`). The rpico draft has no friction and no disturbance torque. The STM32 filter has no input.
- **Next step.** Add a disturbance torque state (random walk) to the filter. Identify `J_total` and `k_d` with a speed-step test. This extends the "Motor Discovery" list at `:1994-2005`.

### 5.9 Noise model of the position filter

- **Problem.** `k_R1` contains the encoder INL (1.2°). INL is a systematic error that depends on angle, not white noise. `Q = diag((k_Q·h)²)` has no off-diagonal terms and its units do not match: `k_Q1·h` with `k_Q1` as an acceleration gives a speed, not a position (`:2163-2191`).
- **Next step.** Calibrate the encoder INL with a slow constant-speed run and a lookup table. Then set `R` from the noise only (σ = 0.068°). Use the white-acceleration model `Q = q·[[h³/3, h²/2], [h²/2, h]]` with `q` from the expected acceleration variance.

### 5.10 Documents out of date

- **Problem.** The design document describes STM32G431, DRV8323, 15 mΩ low-side shunts, 50 kHz PWM and 16.7 kHz control. The livebook uses FDMC86570L and DRV8323. The livebook diode-loss equations are not correct (section 2.6). `smol-servo-modeling.tex` is almost empty and loads the other document's `.bib`.
- **Next step.** Add a "smol-servo RP2350/TMC6200" section with the current values: 5 mΩ load-side shunts, TMC6200 gains, 60 kHz, 20 kHz, NVTFS5C658NL, USB PD 48 V. Recompute the loss table for NVTFS5C658NL with the TMC6200 gate current. Correct the diode-loss equations.

### 5.11 Bus energy at 48 V

- **Problem.** USB PD cannot take energy back (section 2.5). No model gives the bus voltage rise during braking. The margin from 48 V to 60 V parts is small.
- **Next step.** Model the bus as `C·dU/dt = I_source − Σ(d_j · i_j) − I_brake`. Add a brake-resistor control loop on `U_bus`. Measure the bus capacitance on the board.

### 5.12 Smaller items

- `smol_servo_model.c:84` hard-codes 150 MHz. Use the clock value from the SDK.
- `smol_servo_adc.c:200-203` uses literal 150-clock offsets for the sample times. Use `SMOL_SERVO_ADC_ADVANCE_COUNT` (`smol_servo_parameters.h:52`) so that the two values stay equal.
- The ADC scale uses 4095 (`smol_servo_adc.c:422`). The RP2350 ADC full scale is 4096 codes. The error is 0.02 %.
- The 3-phase Wye-Delta comment in `smol_servo_model.c:182-193` has a √3 and a phase error (section 1.5).
- Remove `ssf_kalman.c` or correct it (section 3.3). Do not copy it into the RP2350 firmware as it is.

## 6. Open questions for the user

1. Is there a divider between the TMC6200 current outputs and the ADC pins? The datasheet zero (5VOUT/3) suggests that there is none.
2. Which gain setting do you plan to use with the 2 A stepper?
3. Is the target motor still the 17HS16-2004S-C2?
4. Do you plan to use `ANGLECOM` with the AS5047D delay compensation, or `ANGLEUNC` with a model delay?
5. Which filter type do you want on the RP2350? Options: a full EKF with current states, or a position and speed filter with a separate current loop.
