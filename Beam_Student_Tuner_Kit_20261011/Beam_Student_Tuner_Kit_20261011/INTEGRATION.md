# Firmware-to-tuner contract

Use `#define PIDLINK_EEPROM_ADDR 128` BEFORE including `PidLink.h`, or select a nonoverlapping slot of your own. The example descriptor is included with `#include "TunerDescriptor.h"`. Instantiate `PidLink link;`, start USB `Serial.begin(115200);`, and call `link.begin(TUNER_DESCRIPTOR);` in setup. Keep USB discovery available while stopped and during recoverable sensor faults. Call `link.poll()` frequently in foreground code. Do not perform blocking serial output or I2C inside the scheduling ISR. Do not mix debug text into protocol records.

The example descriptor is MPU6050/I²C/OneShot125. Edit its sensor type/interface/address or CS metadata, output GPIOs, chosen PWM/OneShot125 protocol and measured output rate for your rig. Control timing remains 250 Hz. Both descriptor.example.json and TunerDescriptor.h must agree; advertise only implemented features.

## Required callback responsibilities

| Registration | Your callback signature | Your responsibility |
| --- | --- | --- |
| onPidGet | void(const char *controller) | Reply with all CURRENT runtime gains |
| onPidSet | void(const char *, const PidLinkParams &) | Reject malformed/nonfinite/out-of-range values; preserve omitted fields; apply atomically and then acknowledge actual accepted values |
| onPidReset | void(const char *) | Restore YOUR defaults and reply with actual values |
| onStop | void() | Command physical motor stop, clear controller state, report state |
| onResume | void() | Check your startup/calibration/sensor/level/still gates; never restart automatically; report state |
| onSetTarget | void(float offsetDegrees) | Clamp to your descriptor bounds; offset relative to calibrated neutral |
| onZeroSetpoint | void() | Set neutral at current accepted angle while managing controller state safely |
| onTelemetryRate | bool(uint32_t hz) | Accept only implemented listed rates; true on success, false on rejection |
| onImuSample | bool(PidLinkImuSample &) | Fresh RAW, correctly oriented readings in deg/s and g BEFORE calibration; false on a failed sample |
| onIsStopped | bool() | Actual stopped-output state; required for calibration, save and clear |

For a PID reply call `pidReplyBegin(controller,status)`, `pidReplyParam("kp",value,6)`, similarlyki/kd, then `pidReplyEnd()`. Status constants: APPLIED0, CURRENT1, RESET2, REJECTED4, UNKNOWN_CONTROLLER5. An Apply button remains waiting if you fail to reply or use the wrong controller ID. Use `params.anyMalformed()` and seed proposed gains with existing values before `params.get(...)`; omitted parameters must not become zero. Treat a failed update as an all-or-nothing transaction. Defaults are zero in the example: choose your own tuning approach.

Report `link.state(stopped,canResume,reason)` after STOP, Resume attempt, startup completion and output faults. It does not itself control outputs. Reasons should not contain commas/newlines. PidLink handles discovery, parsing, calibration start/status/save/clear, and telemetry-rate acknowledgement; your callbacks implement the actual hardware and state transitions. Advertise only features you implement.

## Telemetry

At the advertised rate, call `telemetryBegin()`, one `telemetryValue(...)` for EVERY field in EXACT descriptor order, then `telemetryEnd()`. The example order is timestampus, measuredangledeg, targetdeg, errordeg, actualPterm, actualIterm, actualDterm, motorAcommand, motorBcommand, measuredgyrodeg/s, stopped. Timestamp must advance from `micros()` as an unsigned32-bit value, not sample number or milliseconds. Actual P/I/D contributions are controller OUTPUT terms in the declared units, not gain settings. Report applied motor commands, not pulse durations if you declare virtualus. For missing values send an empty field with the header's nonfinite float handling; never omit a column. Request25Hz by default; list50/125/250 only if implemented without starving control or STOP handling. Plotting/captures require each controller's measurement and setpoint fields. Capture telemetry need not run at the control-loop rate.

## Calibration

With onImuSample/onIsStopped registered, PidLink handles stillness capture and calibration persistence. It averages gyro and acceleration, retaining gravity on the dominant acceleration axis and calculating optional attitude offsets when hasAttitude=true. Your sensor callback must return uncalibrated samples consistently; do not subtract saved offsets there. For each control sample apply `link.applyCalibration(sample)` once before consuming corrected data, or use the individual corrected accessors; never both. If recomputing attitude from corrected acceleration, decide explicitly how the saved angle offset is applied to avoid double correction. Gate Resume on `link.calibrationValid()` and `!link.calibrationRunning()`. EEPROM save/clear require stopped outputs. Reserve space for `sizeof(PidLinkCalibration)` at the selected slot. Do not reuse an instructor calibration from another orientation/device. The header does not fuse attitude or schedule control for you.

## Calibration cadence and sensor changes

The supplied header now defaults to a 4000 microsecond calibration sample interval. This matches the 250 Hz MPU6050 output option and is longer than the approximately 2404 microsecond interval of a 416 Hz LSM6DSO32. Sampling requests do not prove that a new hardware conversion occurred: coordinate control and calibration reads with data-ready status and timestamps. Do not collect repeated cached measurements as independent calibration samples, and do not block the control scheduler while waiting for a conversion.

Saved calibration does not identify the physical sensor or its mounting. When changing MPU to LSM, replacing the module, changing measurement units or changing the axis mapping, select a separate nonoverlapping EEPROM region or clear the old calibration before recalibrating. Reserve sizeof(PidLinkCalibration) bytes and verify the selected range fits the Teensy's EEPROM. For example, slots starting at 128 for MPU and 256 for LSM are choices to check against your complete storage map, not automatic sensor detection. Recalibrate level/still, save, reboot and verify status.

## Freshness and stopped recovery

Check for and acquire an available fresh conversion before rejecting an old cache solely for its age. Cache reuse must preserve the original acquisition timestamp. If recovery deliberately discards data captured before a watchdog trip or sensor restart, keep outputs stopped and allow a bounded interval for a new conversion before classifying the intentionally empty cache as a sensor fault. For a 416 Hz LSM, one nominal 4 ms control interval exceeds one sensor output interval; still check actual data-ready status and enforce the chosen recovery timeout. For MPU, use its configured conversion interval and data-ready behavior. Distinguish waiting for a first conversion from failed communication or invalid configuration; never suppress a real fault or resume outputs automatically.

Report the specific fault using a reason without commas or newlines, such as SENSOR_FAULT:NO_FRESH_DATA or SENSOR_FAULT:REG_10_MISMATCH. Include register/expected/actual values or sample age when available. Preserve the latched fault reason when later STOP or Resume commands arrive. Calibration success alone must not clear an unrelated sensor fault.

## Wire commands and replies

ASCII newline-terminated commands, without leading `$`: `SYS,DESCRIBE`, `SYS,ID`, `SYS,PING`; `PID,GET,beam`; `PID,SET,beam,kp=0,ki=0,kd=0`; `PID,RESET,beam`; `OUT,STOP`; `OUT,RESUME`; `SET,TARGET,2`; `SET,ZERO`; `TELEM,RATE,25`; `CAL,START`, `CAL,STATUS`, `CAL,SAVE`, `CAL,CLEAR`.

Header emits `$D,index,total,jsonChunk` (zero-based chunks), `$I,name,version,descriptorHash`, `$P,beam,status,kp=...,ki=...,kd=...`, `$T,...fields`, `$S,STATE,stopped=1,can_resume=0,reason=...`, `$K,...` and `$E,...`. Calibration replies use `$C,...` and are generated by the header. Descriptor hash is computed by the header from the exact JSON. Change version/descriptor when units/layout change; reconnect the app after reflashing. The `rig` must be `lab`, protocol1, with valid IDs and correct field references. If scaling displayed gains, describe those units and convert ALL SET/GET/RESET paths consistently; the example has no scaling.
