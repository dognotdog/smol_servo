# AS5047D detection failure after a warm reset

Date: 2026-11-09. Board: smol-servo (RP2350, TMC6200, AS5047D on SPI0). Files: `firmware_rpico/src/as5047d.c`, `as5047d.h`.

## Symptom

- In 3 of 22 boot logs, the console showed `AS5047D detected: 0`.
- In a failed boot, the NOP answer was `0xC000` (EF set) and ERRFL was `0x4004` (EF + PARERR). The PROG and SETTINGS1 reads were correct.
- `smol_mag_init()` does not start the MAG DMA when the detection fails. The servo loop then has no angle data.

## Cause

A warm reset of the RP2350 cuts an active MAG SPI frame. The AS5047D keeps its power and keeps that frame open. The first CSn rising edge at the next init ends the cut frame, and the AS5047D sets PARERR.

Details:

1. A warm reset (`make swd_flash`, `make swd_reset`, watchdog) does not remove VBUS. The 3V3 rail and the AS5047D stay on. The boot log shows `VBUSOK 1` and a PD contract after approx. 351 ms. A hub power cycle shows `VBUSOK 0` and approx. 930 ms.
2. `smol_pwm_start()` runs at the first zero calibration (`s`). After that, PWM slice 10 starts one MAG frame every 12.5 µs. A frame is 16 bits at 1.43 MHz (11.2 µs). Thus a frame is active for approx. 90% of the time.
3. The board has no pull-up on MAG_NSS (CSn). The schematic net MAG_NSS connects only J7-4/J1-4 and U4 pin 1. After reset, the RP2350 pad has its pull-down on (pad register `0x116`: ISO, PDE, IE off). Thus CSn stays low after the reset, and the AS5047D does not see the end of the frame.
4. At the next init, the first CSn rising edge ends the cut frame. The frame has the wrong number of bits, and the AS5047D sets PARERR. The next answer has EF set.
5. The old detection sent NOP first. The NOP answer had EF set (`0xC000`), so the check `rx[0] == 0` failed. The old detection then read ERRFL, which cleared the error. Thus the PROG and SETTINGS1 reads were correct.

All 3 failed logs show `VBUSOK 1`. In the run before each of them, the user did a zero calibration.

## Evidence

Each test run sends `s` approx. 4.5 s after the start, so the MAG DMA runs at the next reset. The test scripts are in the scratchpad of this session (`resetloop_s.sh`, `logsend.py`).

| Firmware | Reset | Boots | `detected: 1` | Stale PARERR |
|---|---|---|---|---|
| Original `as5047d.c`, no `s` | SWD reset | 30 | 30 | (not visible) |
| Original `as5047d.c`, with `s` | SWD reset | 15 | 9 | (not visible) |
| Fix, with `s` | SWD reset | 25 | 25 (attempt 1) | 13 of 24 boots, cleared |
| Fix, with `s` | Hub power cycle (port 1) | 22 | 22 (attempt 1) | 0 |

- Original code with `s`: 6 failures in the 8 boots that came after a run with MAG DMA. 0 failures in the 6 boots that came after a failed run (no MAG DMA). The failed values were the same as in the original logs (`0xC000`, `0x4004`, `0x0000`, `0x8001`).
- With the fix, the first answer (`prev`) after a warm reset is angle data from the previous run, for example `0x0fdd` or `0x4fdf`. This shows that the AS5047D kept its state across the reset. After a power cycle, `prev` is always `0x0000`.
- These tests did not cause the failure:
  - 200 TMC6200 state reads before the MAG init (crosstalk on J7), 10 boots.
  - The old pin sequence without `s`, 45 boots (30 SWD resets, 15 `make swd_flash`).

## Datasheet checks (no problem found)

- SPI mode 1 (CPOL=0, CPHA=1): correct.
- Clock 1.43 MHz. The maximum is 10 MHz. tL (350 ns) is the limit for this clock.
- tCSn (350 ns minimum): the gap between detection frames is more than 1 µs (`sleep_us(1)`).
- tpon (10 ms): the MAG init is at approx. 0.42 s after boot.
- The old code used the answer of the correct frame. The AS5047D sends the answer to a command in the next frame.

## Fix

In `as5047d_spi_init()`:

1. Set CLK and MOSI to a driven low with SIO.
2. Set CSn high with SIO. This ends a cut frame before the first real frame.
3. Wait 10 µs (more than tCSn).
4. Change the pins to the SPI function. The levels are the same as the SPI idle levels, so this causes no edge.

In `as5047d_detect()`:

1. Read ERRFL first. This read clears the stale error. If the value is not 0, write it to the log at INF level (`ERRFL 0x4004 before the detection frames (cleared)`). This is not a detection failure.
2. Read NOP, ERRFL, PROG and SETTINGS1. All must have EF=0 and the expected values.
3. Do a maximum of 3 attempts. Each failed attempt writes a WRN line with all values. In the tests, attempt 1 was always sufficient.

Other changes:

- `_read_registers()` always returned -1 for more than 1 register (`read_result != numregs` after a 1-word transfer). It now returns 0 on success. It also gives the answer of the first frame (`prev`) for the log.
- `as5047d_spi_init()` now returns 0.
- New register names in `as5047d.h`: `MAG_REG_NOP`, `MAG_REG_PROG`, `MAG_REG_SETTINGS1`.

## Remaining items

- A hardware pull-up on MAG_NSS does not remove the stale error. The pull-up ends the cut frame at the reset, and the AS5047D sets PARERR then. The ERRFL read at init is necessary in both cases.
- Other code can read the AS5047D before `as5047d_detect()` in the future. That code must also clear ERRFL first after a warm reset.
- Not related to the AS5047D: in 3 of 22 hub power cycles, the first boot showed `fusb302_tx failed: i2c_write_blocking_until()=2` at approx. 10 ms. The board then started again. The second boot was correct.
