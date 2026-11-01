# RCA-MM36110-15kHz-Bridge

Arduino Nano I2C man-in-the-middle between the RCA MM36110 monitor MCU and its Toshiba TA1300AN (0x24).
It suppresses the spurious `reg 0x00 = 0x80` (33.75 kHz) write the MCU issues roughly every 7.2 s while a
15 kHz source is shown on VGA.

## Suppression logic

The `0x80` write is replaced with `0x00` only when **all** of these hold:

1. The VGA pin is high.
2. The bridge's own background poll of the TA1300AN read registers has reported a 15 kHz horizontal
   frequency (read byte 1: H-OUT set, H FREQ DET `Y` in 30..45, i.e. ~12.6-18.9 kHz; 15.75 kHz reads ~37,
   31.5 kHz ~75, 33.75 kHz ~80) for `H15K_STABLE_COUNT` consecutive polls, and the last valid poll is
   less than `H15K_STALE_MS` old.

Anything else (no sync, no ACK, 31/33 kHz, stale data) passes the write through unchanged. If a write was
suppressed and the IC then reports a non-15 kHz input within `PENDING_REPLAY_MS`, the `0x80` write is replayed,
so a legitimate mode change is only delayed. The monitor's own `0x00` correction clears the pending replay.
Thresholds are `#define`s at the top of the sketch.
