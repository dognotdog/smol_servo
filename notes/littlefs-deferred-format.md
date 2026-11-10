# littlefs: deferred format (2026-11-09)

## Problem

The board gets its only power from USB-C VBUS. A flash erase disables the core 0 interrupts. A 4 KiB sector erase can take several 10 ms, and the worst case can be longer. During this time, core 0 does not run `smol_usb_pd_run()` and cannot answer PD messages.

Before this change, `smol_fs_init()` formatted the FS when the mount failed. This occurs on the first boot and when the FS is corrupt. The format occurred directly after `smol_usb_pd_run_startup()`. At that time the PD negotiation can still continue, for example an EPR entry. A late answer can cause a Hard Reset, and then the board loses VBUS.

The build also sets `PICO_FLASH_ASSUME_CORE1_SAFE=1`. Thus `flash_safe_execute()` does not stop core 1. A flash write is safe only after core 1 runs from RAM only (`smol_core1_is_ram_only()`).

## Design

1. `smol_fs_init()` only mounts. A mount only reads the flash.
2. If the mount fails, `smol_fs_init()` sets a pending format and logs a warning. It does not erase.
3. `main()` calls `smol_config_load()` only when the FS is mounted. Thus a normal boot loads the config at once. On the first boot, or with a corrupt FS, the config load waits for the format. Until then, the driver uses its default gain.
4. The core 0 main loop calls `smol_fs_run()`. It does the format only when all these conditions are true:
   - A PD contract exists: `smol_usb_pd_contract_mv() > 0`.
   - The contract voltage did not change for `FS_FORMAT_PD_SETTLE_US` (3 s).
   - Core 1 runs from RAM only: `smol_core1_is_ram_only()`.
5. After the format, `smol_fs_run()` mounts the FS and calls `smol_config_load()`. A new FS has no `/config.json`, so the defaults apply.
6. A failed format, or a failed mount after the format, is fatal. `smol_fs_run()` calls `panic()`. The config is then not loaded.

## Driver enable gate

`smol_drv_enable()` refuses to enable the driver until `smol_config_is_loaded()` is true. The config sets the current sense gain, so the driver must not run with a gain that is not known.

- `smol_config_is_loaded()` is true only after a `smol_config_load()` that returned 0. That is: the file was read with no errors, or there is no file (defaults), and the gain was applied.
- A JSON error, a key error, or a gain that was not applied makes it false. Correct the config with `config set` and `config save`, then `config load`.
- `smol_config_load()` is not allowed while the driver is enabled, because the gain change is not allowed then. It then does not change the loaded state.

## Why a settle time

The PD code has no public "negotiation idle" signal. `smol_usb_pd_contract_mv()` is above 0 also during a contract change, for example during the EPR entry after the first SPR contract. During the EPR entry the value can stay at the old voltage until PS_RDY. The EPR entry delay (100 ms), SinkEPREnterTimer (550 ms) and the EPR PSTransitionTimer (1020 ms) give approximately 1.7 s. Thus the settle time is 3 s.

A better signal is a function in `smol_servo_usb_pd.c` that returns true for state `SMOL_USB_PD_READY` with no traffic for some time. This change does not add it, because `smol_servo.h` is in use by another work session.

## Remaining risk

- `lfs_format()` erases 2 blocks (the superblock pair) and programs some pages. During the format, the main loop does not run. In EPR mode, the sink must send EPR_KeepAlive. The firmware sends it after 300 ms without traffic. The Source timeout (SourceEPRKeepAliveTimer) is 750 ms minimum. A format of approximately 100 ms is thus safe. A very slow erase can be a problem.
- If the Source sends a message during the format, the FUSB302 sends GoodCRC, but the firmware answers late. The Source can then send Soft_Reset. A Soft_Reset does not remove VBUS.
- Without a PD contract (for example a 5 V USB-C supply without PD), the format does not occur, and the FS stays unmounted. `smol_config_save()` then fails with "no filesystem". The driver enable stays blocked. The driver also needs a PD contract, so this is not a new limit.
- After a `panic()`, core 0 does not answer PD messages. With an EPR contract, the Source sends Hard Reset when the EPR_KeepAlive stops. The board then starts again and tries the format again, approximately each 5 s.

## Test

1. Flash the firmware.
2. Erase the superblock pair of the FS (the first 8 KiB at `0x10180000`). The FS is at `0x10180000` to `0x10200000`. Use an SPR contract (for example 20 V), because openocd halts the core during the erase, and the core then cannot send EPR_KeepAlive.
3. Reset the board, or power-cycle the port, with a 20 V source.
4. Expected log:
   - `FS mount failed (-84), format and config load deferred ...`
   - no `config:` line at this time
   - the PD contract, with no Hard Reset
   - approximately 3 s after the last contract change: `FS format start, PD contract 20000 mV`
   - `FS formatted in N ms and mounted ...`
   - `config: no /config.json, defaults in use`
5. Reset again. Expected log: `FS mounted, 128 blocks of 4096 bytes`, and no format.

## Test result (2026-11-09, 20 V SPR contract, hub port 1)

- Before the test, a backup of the FS region (512 KiB) was made with openocd `dump_image`. After the test, the backup was written again and verified. `/config.json` (40 bytes) is back.
- Erase of the first 8 KiB, then a port power cycle:
  - `FS mount failed (-84), format deferred ...` at 0.99 s.
  - `FS format start, PD contract 20000 mV` at 3.99 s.
  - `FS formatted in 62 ms and mounted`, then `config: no /config.json, defaults in use`.
  - No PD error after the format. VBUS stayed at 20 V.
- SWD reset with the new FS: `FS mounted`, no format.
- Power cycle after the restore: `config: /config.json read, 40 bytes, 0 key errors`.

## Test result 2: config load after the format, enable gate (2026-11-09)

- Normal boot (SWD flash): `config: /config.json read` at 0.41 s. `drv_en` enabled the driver.
- Erase of the first 8 KiB, then a port power cycle:
  - `FS mount failed (-84), format and config load deferred ...` at 0.99 s. No `config:` line at this time.
  - `drv_en` at 1.05 s: `TMC6200 enable: config not loaded`.
  - `FS formatted in 63 ms and mounted` at 4.06 s, then `config: no /config.json, defaults in use`.
  - `drv_en` at 6.70 s: `TMC6200 enabled, VS 20000 mV`.
- After the restore of the backup: `config: /config.json read, 40 bytes`. `cfg_load` with the driver enabled: `config: load not allowed while the driver is enabled`. `cfg_load` after `drv_dis`: the config was read again.
- Not tested: the `panic()` after a failed format. There is no simple way to make the format fail.

Other observations (not caused by the FS change):

- After the first `make swd_flash`, core 0 stopped in a HardFault before the core 1 launch. The fault was in the USBCTRL interrupt, in `dcd_event_handler()` (TinyUSB), a precise bus fault at `0xf0000000`. A power cycle and a later SWD reset did not show the fault.
- Two of the power-cycle boots had a PD error at the start: `I_RETRYFAIL` and a Hard Reset, or `No USB PD source detected!` at 7 ms and then `no response to Request`. In both boots the sink got the 20 V contract after the retry.
