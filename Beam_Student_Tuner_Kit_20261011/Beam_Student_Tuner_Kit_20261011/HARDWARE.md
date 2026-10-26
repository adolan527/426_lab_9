# Lab 9 hardware and interface choices

Choose one IMU and verify its physical module labels and the rig schematic. Use compatible module power and 3.3 V Teensy signal levels with common ground; connector order differs between modules.

| Module | Bus checks for your wiring | Arduino sensor library |
| --- | --- | --- |
| HiLetgo MPU6050 / GY-521 | I²C: identify SDA/SCL; choose compatible Teensy bus pins; verify address and module power/pinout | Adafruit MPU6050 |
| Adafruit MPU6050 | I²C: choose compatible Teensy bus pins using this breakout’s pinout | Adafruit MPU6050 |
| Adafruit LSM6DSO32 | SPI: identify SCK/MOSI/MISO/CS; select compatible Teensy pins and document your CS GPIO | Adafruit LSM6DS, LSM6DSO32 example |

Install Adafruit Unified Sensor and Adafruit BusIO for either library path. For the LSM SPI course setup, use Mode 0, MSB first, 4 MHz. Read WHO_AM_I register 0x0F and expect 0x6C before proceeding to sensor samples. Mode 3 produced an invalid identity in the reported Teensy test; changing to Mode 0 restored reads. This is the verified course configuration, not a claim that every Mode 3 setup fails. Confirm the actual library/driver transactions and CS wiring.

Adafruit sensor events use rad/s and m/s². PidLink calibration samples require deg/s and g: convert once. Draw the actual +X/+Y directions relative to each motor, +Z/board face, beam pivot axis, viewing direction and positive tilt. Motor CW/CCW rotation does not establish the sensor frame or feedback sign.

Choose compatible Teensy GPIOs for your ESC signals using the Teensy pinout and the requirements of your selected PWM/OneShot125 driver. Keep a common ESC/Teensy signal ground. Record motor A/B physical sides, measured CW/CCW rotation, matched propellers and your own GPIO map. Keep sensor, output and other module assignments compatible; do not copy pins from unrelated reference firmware.

## Choose PWM or OneShot125 before asking AI for code

Verify that both ESCs support your selection and output frame rate. PWM pulse widths typically span 1000..2000 µs. OneShot125 uses 125..250 µs; if your code keeps command counts 1000..2000, divide by 8 only in that output driver. Those counts are not OneShot pulse durations. Implement and measure your own driver, stopped waveform and arming behavior for the chosen protocol. Do not substitute generic duty-cycle output or silently reuse the diagnostic app firmware’s protocol.

The control loop must run at 250 Hz, nominal 4 ms. Select/document sensor sampling, range, filtering and scheduling for that rate. Choose an ESC-supported output frame rate separately; if using 250 Hz output frames, verify the ESC accepts 4 ms frames with the selected pulse widths. Measure control and output timing, overruns and worst-case sensor/USB handling. Keep discovery and STOP responsive; missed control deadlines must stop outputs even if a hardware timer keeps generating pulses.

The example descriptor uses MPU6050/I²C, OneShot125 and 250 Hz output frames, with no output pin assignments. Add your own chosen GPIO metadata. Change it to your actual sensor, bus, GPIOs, `output_protocol` (`pwm` or `oneshot125`), measured `output_rate_hz`, units and implemented features. The app interface does not choose the sensor or generate a motor driver for you.

## Sensor timing and calibration

MPU6050 starting option: DLPF enabled, divider 3 for 250 Hz register updates, approximately 42 Hz gyro / 44 Hz accelerometer filtering. LSM6DSO32 starting option: both sensors at 416 Hz, gyro LPF1 approximately 48 Hz and accelerometer LPF2 ODR/10 (41.6 Hz). Explicitly configure and verify the chosen settings. These are starting options to evaluate rather than guaranteed optimal tuning. Sensor output timing and the 250 Hz control schedule are separate.

Calibration requests default to 4000 microsecond spacing in this kit. Verify actual sample freshness for either sensor; do not assume the library's event return value alone proves freshness or communication health. Adafruit events are already in physical SI units; register reads require conversion using the exact sensor and configured range. Recalibrate after changing sensor or mounting, and isolate saved calibration as described in INTEGRATION.md.

## Operating limits and stopped startup

Confirm instructor-approved limits before powered testing. The interface reference uses base command 1300, running bounds 1100..1650, stopped 1000, target ±10°, cutoff 45° relative to target and Resume within 10° and below 20°/s with valid calibration/fresh data. These are command-space references, not PID gains or a mixer. Translate commands into the chosen protocol’s pulse widths and verify them against ESC setup; descriptor labels and telemetry must state truthful units.

Start all PID gains at zero and both outputs stopped. Failed/stale sensor data and missed control deadlines must stop outputs without automatic restart. Keep a physical cutoff available. Complete USB-only acceptance with ESC power disconnected before the approved restrained-beam procedure. Props may remain fitted during those USB-only checks; follow course propeller rules for all other work.

## References

- MPU library/setup: https://learn.adafruit.com/mpu6050-6-dof-accelerometer-and-gyro/arduino
- LSM library/setup: https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/arduino
- LSM6DSO32 identity register, §9.11: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
- Typical protocol widths: https://docs.px4.io/v1.14/en/peripherals/oneshot
