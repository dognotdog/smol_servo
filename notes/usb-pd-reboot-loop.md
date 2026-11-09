# USB PD reboot loop after SWD reset (2026-11-09)

## Symptom

After `make swd_flash` with an active 20 V contract, the board did not get a new contract. VBUS went off, and the board started again approximately every 0.8 s. A port power cycle on the Acroname hub sometimes recovered the board. On 2026-11-09 a port power cycle did not recover it.

## Reset cause

The firmware now logs `POWMAN CHIP_RESET` and `WATCHDOG REASON` at each boot.

- Each boot in the loop had `CHIP_RESET = 0x00010000` (`HAD_POR`). `WATCHDOG REASON` was 0.
- Thus the RP2350 did a power-on reset. It was not the watchdog, and it was not a fault.
- The board gets its power only from VBUS. The probe does not supply power.
- The hub measured VBUS on for approximately 75 ms in each cycle, at approximately 40 mA. Then VBUS was off for approximately 0.7 s. The 0.16 V to 0.2 V between the pulses is the slow discharge of the board VBUS capacitors.
- The current was low, so the hub overcurrent protection did not cause the loop.

Note: the 30 ms period in the older report came from the firmware timestamps. Each boot lived for only 30 ms to 40 ms. The host timestamps show the 0.8 s period.

## Cause

There are two parts.

1. After an SWD reset, only the RP2350 resets. The Source keeps its Explicit Contract, so it does not send Source_Capabilities. The sink waited for tTypeCSinkWaitCap (620 ms) and then sent Hard Reset. The Hard Reset removed VBUS, so the board lost power.
2. After the power-on, the RP2350 starts approximately 40 ms after VBUS is on. The Source sends its first Source_Capabilities before the FUSB302 is ready. After a Hard Reset, this Source then sends Hard Reset again (approximately 25 ms to 35 ms after the board starts). It does not send Source_Capabilities again. This makes the loop.

Blocking log output made part 2 worse:

- The old firmware wrote approximately 7 log lines before it set AUTO_CRC. Each line is a blocking UART write of approximately 3.5 ms to 9 ms at 115200 baud. AUTO_CRC was on only at 26.5 ms.
- After an Accept, the RX log line delayed the Source_Capabilities handling. A trace showed the Request at 46 ms. That is after tSenderResponse, so the Source sent Hard Reset at 74 ms.

The Source behavior after a Hard Reset is an inference from the timing and from a FUSB302 interrupt trace. The trace showed `I_HARDRST` with no RX message and no `I_GCRCSENT` before it.

## Fix

- `main()` starts the PD sink before all other init and log output. Only `stdio_init_all()`, the LED GPIO and the TMC6200 SPI init come before it.
- `smol_usb_pd_init()` sends Soft_Reset just after the attach. The Source sends Accept and then Source_Capabilities. This works when the Source has a contract (SWD reset) and also after a power-on. VBUS stays on.
- If the startup Soft_Reset gets no Accept, the sink waits for Source_Capabilities. It does not send Hard Reset at once.
- At the SinkWaitCapTimer timeout, the sink sends Hard Reset (spec behavior). An older version of the fix sent Soft_Reset first. The startup Soft_Reset made that redundant, so it is removed. The other paths to the wait for Source_Capabilities (attach by CC polling, received Hard Reset) start with a new Source, and that Source sends Source_Capabilities.
- `smol_usb_pd_run_startup()` runs only the PD sink until the contract is complete and there is no PD traffic for 250 ms (2 s max). Core 1 starts after this.
- The PD log output goes into a queue. `smol_usb_pd_run()` prints at most one line for each call, after the RX handling. The time in each line is the time of the event.
- `fusb302_chip_init()` writes no log output. `smol_usb_pd_init()` prints the device ID and VBUSOK after the attach.

## Test results (bench, Acroname USBHub3c port 1)

- `make swd_flash` with an active 20 V contract: 6 of 6 pass. "PD soft reset accepted" at 27 ms. PS_RDY at 118 ms. VBUS stayed between 19.994 V and 20.006 V.
- Hard Reset from the hub (`hub.pd[1].request(0)`): 3 of 3 pass. VBUS goes off one time. After the power-on, PS_RDY comes at approximately 140 ms.
- Port power cycle: pass. PS_RDY at 132 ms.
- After the removal of the Soft_Reset at the SinkWaitCapTimer timeout: `make swd_flash` 3 of 3 pass (PS_RDY at 118 ms to 121 ms, VBUS between 19.995 V and 20.006 V). Hard Reset from the hub 3 of 3 pass (PS_RDY at approximately 138 ms).

The first fix had only the Soft_Reset at the SinkWaitCapTimer timeout. It passed 4 of 4 `swd_flash` tests (PS_RDY at 745 ms). But the hub Hard Reset test still looped. The startup Soft_Reset and the log queue fixed that.

## Bench procedure

- Use a venv with `brainstem` and `pyserial`. Port 1 VBUS: `hub.hub.port[1].getVbusVoltage()`.
- To send a Hard Reset to the board: `hub.pd[1].request(0)`.
- To flash a board that is in the loop: run OpenOCD `init; halt; program <elf> verify` in a retry loop. One try can halt the CPU in a 75 ms power window. The halt can keep VBUS on, because the Source gets no GoodCRC. Up to 167 tries were necessary.
- After a long halt, the hub Source can end PD communication (`I_RETRYFAIL` on Soft_Reset, VBUS at 5 V). A port power cycle clears this.

## Open items

- Not tested: SWD reset with an EPR contract. After a Soft_Reset in EPR Mode, the Source can send EPR_Source_Capabilities. The sink starts with `_epr_mode = false` and ignores EPR_Source_Capabilities in that state. The Source then can send Hard Reset, and VBUS goes off.
- The startup Soft_Reset does not check SinkTxOk (BC_LVL was 1 or 2 at some power-ons). The hub accepted it each time. Check the PD 3.2 collision avoidance rules for Soft_Reset.
- After startup, `picobug_process_core1_queue()` prints all core 1 lines with blocking writes in the main loop. This can still delay a PD response (for example EPR_KeepAlive or a PPS Request).
- The I2C bus to the FUSB302 runs at 100 kHz. A faster bus can make the attach faster than 12 ms. First, check the pull-up resistors on the board.
