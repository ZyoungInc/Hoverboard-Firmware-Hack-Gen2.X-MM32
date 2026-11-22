# First diagnostic burn attempt — 2026-11-03

Repository baseline: `182919ead170b160e82eaa428ae9c08076547af2`.
The user authorized full-chip erase of the connected MM32SPIN05PFOP and programming only `build/spin05/diagnostic.hex`, accepting the unverified SWD voltage margin. The user reported a stable 36 V bus, 5 V MCU rail and limited supply with SW held. This host cannot independently read either supply voltage from the DAPLink.

The repo's `tools/mm32flash.py` erases 1 KB pages, so it was not used for this full-chip operation. MindMotion's `MM32SPIN0x_DFP` 1.0.8 pack assigns `Flash/MM32SPIN05_32.FLM` to `MM32SPIN05PF`. pyOCD exposes its 32 KB flash region, 1 KB sectors and nonzero `pc_eraseAll`. The connected core read CPUID `0x410CC200` and DBGMCU IDCODE `0xCC4460B1` using target `mm32spin05pf` at 100 kHz. The 1,688-byte diagnostic HEX converted back to BIN exactly matches `diagnostic.bin` and ends at `0x08000698`.

Erase command (full log: `build/spin05/burn-chip-erase.log`):

```sh
timeout 90s .venv/bin/pyocd erase --no-config -t mm32spin05pf \
  -u F45C7948E30E29288708BD68764F9178 -f 100000 -M attach \
  -O auto_unlock=false -O resume_on_disconnect=false --chip -v
```

pyOCD reported `Erasing chip...` then `Chip erase complete` with process exit code 0. Immediately afterward, a separate read-only session using the same pack target read all 32,768 flash bytes in 1 KB blocks. All bytes were `0x00`, including the first and last 16 bytes. The flash status register read `0x00000020` (EOP set, no WRPRTERR or PGERR). CPUID and IDCODE were unchanged. Full log: `build/spin05/burn-erase-verify.log`.

**The initial erase verification failed. At that point, no programming, further erase/unlock attempt, reset, or UART test was performed.** The flash's physical contents were then unknown: zero reads could have been caused by access protection or a failed erase. Do not interpret pyOCD's success message alone as proof that the flash is writable. No diagnostic image was written.

## Follow-up on the same date

At the user's request to retry unlock and erase, a SWD system reset and halt was performed (`build/spin05/retry-reset-read.log`). After reset, CPUID and IDCODE were unchanged, `FLASH_SR=0`, `FLASH_CR=0x80`, `FLASH_OBR=0x03FFFC1C`, `FLASH_WRPR=0xFFFFFFFF`, the 16 option bytes read `a55affffffffffffffffffffffffffff`, and both first and last 16 flash bytes read `0xFF`. This demonstrates that the earlier all-zero readback was transient across reset; it does not prove that every byte is erased.

The next read-only full 32 KB verification (`build/spin05/retry-pre-erase-full-verify.log`) failed mid-transfer with `SWD/JTAG communication failure (Unexpected ACK '0')`. **No second erase, option-byte modification, programming, or UART test was performed after this communication failure.** The user's stop-on-SWD-error condition applies. The connection must be stable for a complete flash read before any further destructive operation or programming.

## Second retry

At 50 kHz, 64-byte SWD transfers read all 32,768 flash bytes as `0xFF` without errors (`build/spin05/retry2-flash-read.log`). A second official pack `EraseAll` was then executed using the same `pyocd erase --chip` command as above but at 50 kHz (`build/spin05/retry2-chip-erase.log`). After a SWD reset and halt, a second complete read found zero non-`0xFF` bytes; CPUID and IDCODE matched, and the option bytes read `a55affffffffffffffffffffffffffff` (`build/spin05/retry2-erase-verify.log`). No manual option-byte unlock or other-chip algorithm was used.

The user-authorized diagnostic HEX was then loaded with the MindMotion pack target at 50 kHz using `pyocd load -e chip --no-reset`; pyOCD reported `Erased chip, programmed 2048 bytes (2 pages)` and exit code 0 (`build/spin05/retry2-diagnostic-load.log`). This load operation performed another full-chip erase as requested by `-e chip` before writing. The following independent readback verification could not open the target: DAPLink reported `SWD/JTAG communication failure (No ACK)` during `DebugPortSetup` (`build/spin05/retry2-diagnostic-verify.log`). USB still lists the DAPLink probe. **Flash contents, MCU restart and UART output are unverified; no further target operation was attempted after the SWD failure.**

## Diagnostic readback and run

The user measured 3.3 V at the MCU VDD decoupling capacitor. At 50 kHz, the pack target still could not connect (`build/spin05/post-vdd33-swd-reconnect.log`). The generic read-only `cortex_m` target at 10 kHz connected and read the expected vector table (`build/spin05/post-vdd33-generic-reconnect.log`). It read all 1,688 diagnostic image bytes without mismatches; both local and target SHA-256 were `c71ef6098d9c99357e29d3f7c21275cefcb65d4a33e6e8da10e3b2657622b512`. Sixteen bytes after the image and at the config page start read `0xFF` (`build/spin05/post-vdd33-diagnostic-readback.log`).

The generic target then reset, halted and resumed the MCU. RAM diagnostic status magic was `0x47443234`, with milliseconds advancing from 1000 to 2000, ADC valid set, raw VBAT ADC 947/946, Hall bits 7, PB5 button 0, PB2 latch output 1; core state was RUNNING (`build/spin05/post-vdd33-diagnostic-run.log`). The user connected the target UART TX to the DAPLink VCOM RX. `/dev/ttyACM0` at 115200 8N1 captured periodic diagnostic lines every 500 ms (`build/spin05/diagnostic-uart-capture.log`, `build/spin05/diagnostic-uart-clean.log`). The first read contained some undecodable buffered bytes, followed by stable valid text. Button, self-hold and Hall transitions have not yet been tested.

The user subsequently released the SW short, reported that the board's 5 V rail remained present, and UART continued to stream (`build/spin05/diagnostic-sw-released.log`). PB2 stayed high and PB5 raw input stayed low in that capture. Live SWD GPIO reads were consistent: GPIOB IDR `0x4C` (PB5 low, PB2 high), ODR `0x44` (PB2 high), CRL `0x4D444104` (`build/spin05/diagnostic-gpio-live.log`). The user explicitly confirmed PB5 is the button input and asked for no further PB5 measurement. No PB5 polarity transition was observed. Hall input state remains static at `7`, so Hall operation has not been verified.

The user then clarified that the motor/wheel is not connected. A 30-second UART capture showed Hall `7` throughout (`build/spin05/diagnostic-hall-handturn.log`); this has no bearing on Hall sensor wiring because its connector is open. No Hall test or motor movement was performed.
