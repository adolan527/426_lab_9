# Student integration acceptance checklist

## Before powered testing

Complete these checks using USB with ESC power disconnected; props may remain fitted. Use approved instruments for waveform checks.

- Code compiles for Teensy4.0; no instructor firmware solution included.
- Confirm your chosen IMU communicates and yields fresh, correctly scaled samples; LSM6DSO32 SPI WHO_AM_I must read 0x6C with the verified Mode 0/MSB-first/4 MHz settings.
- App beam/lab page discovers correct name/version, selected sensor/bus, output protocol/rate, controller IDs, units and zero defaults.
- Read returns actual gains; Apply confirms accepted values; Reset returns your zero-gain defaults.
- A partial gain SET preserves omitted values; malformed, nonfinite, negative and excessive values reject the whole update without altering gains.
- Telemetry field count/order/timestamp/units match descriptor; measured and target traces update. Actual term traces represent output contributions.
- STOP changes actual motor output state and clears accumulated control state; app reports stopped. Repeated STOP is harmless.
- Resume rejects invalid calibration, moving/tilted rig, sensor faults and incomplete startup. An accepted Resume requires an explicit new command.
- Calibration while stopped produces offsets in correct units; save/reboot/read preserves valid calibration; active-output calibration/save/clear reject. No double subtraction.
- For the selected MPU or LSM, verify calibration sampling uses the configured conversion rate and the kit's 4000 microsecond request interval; no fabricated or repeated stale samples are counted as independent readings.
- With ESC power disconnected, complete calibration, save, reboot, read status and a gated Resume/STOP check without false sensor faults. Repeat this acceptance sequence for each supported IMU implementation; one sensor's result does not validate the other.
- With ESC power disconnected, exercise any implemented watchdog/sensor-restart recovery. Deliberately discarded data must lead to a bounded stopped wait for a new conversion, followed by either fresh-data eligibility for explicit Resume or an informative timeout/fault. Outputs must not restart automatically.
- Change or clear calibration storage before swapping sensor type or mounting; verify the previous configuration's offsets are not silently reused.
- Fault reasons include a useful cause and use no commas/newlines; a later STOP must not hide a latched fault cause.
- Listed telemetry rates work; unsupported rates reject; discovery and STOP remain responsive during telemetry.
- Sensor read/config/stale-data faults and missed-control deadlines command stop. Restoring a sensor alone does not restart outputs.
- Output pin/protocol mapping matches assignment; demonstrate the chosen PWM or OneShot125 pulse widths and output frame timing without ESC power before powered operation.
- Measure the 250 Hz control-loop timing (4 ms nominal), chosen output frame timing, overruns and worst-case USB/sensor handling. Confirm descriptor rates match measured implementation and that STOP remains responsive.
## Approved powered test and live TA demo

After the USB checks, follow the instructor’s restrained-beam procedure and propeller rules.

- Demonstrate correct physical feedback direction, bounded response and STOP on the actual rig.
- Keep your source, descriptor, library versions and checks available on your laptop for the live TA demonstration. Show physical feedback sign and a bounded response on the actual rig after the approved checks. Nothing is uploaded to Moodle; no report, source ZIP or capture submission is required.
