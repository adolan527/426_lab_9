# Diagnose beam bias before tuning

An imbalance that moves with a propeller strongly suggests a propeller or
propeller/rotation mismatch. It does not establish a motor or ESC fault. Check
diameter, pitch, blade count, CW/CCW handedness, mounting orientation, damage,
hub seating and attachment with power disconnected. Use matched, undamaged
propellers, each appropriate for its motor's actual rotation and installed to
produce upward thrust. CW/CCW props cannot simply be exchanged between opposite
rotation motors as an equivalent test. Follow the propeller manufacturer's
orientation instructions; flipping a propeller is not a substitute for matching
its handedness to rotation.

## Separate command bias from thrust bias

1. With ESC power disconnected, calibrate the level/still sensor and verify the
   displayed angle: raising motor A must increase angle. This sketch assumes
   sensor +Y toward A, +Z upward when level, and upward thrust at both ends.
2. Read back gains after Apply. With `kp=ki=kd=0`, running commands should be
   `motor_a=1300`, `motor_b=1300` and `pid=0`. STOP must give 1000 on both.
   Resume remains gated by calibration, freshness, level and stillness checks.
   Verify the actual D4/D23 waveforms using an approved instrument: equal pulse
   widths and 4 ms frames. Telemetry reports commands, not measured RPM or thrust.
3. Check correction direction without ESC power: with positive Kp and Ki/Kd zero,
   raise A slightly while within the allowed angle range. A's pulse should decrease
   and B's increase. The prior `MIXER_SIGN=-1` reversed this correction; the sketch
   now uses +1. If measured angle or correction disagrees, stop and resolve the
   mounting/output mapping before powered testing.
4. After these checks, use only the instructor-approved restrained-beam test.
   If equal measured commands still give unequal thrust, correct the propeller
   mismatch first. Check ESC configuration/calibration according to its maker's
   procedure if the problem remains with a motor/ESC after fitting suitable props.
   Also check pivot friction and unequal arm lengths or mass distribution.

## Residual steady error

Equal pulse widths do not guarantee equal thrust. Once hardware and feedback
direction are verified, proportional control alone can leave a steady error under
a constant disturbance. Integral action can correct a modest residual imbalance
within the available actuator authority. This sketch limits integral error to
50 degree-seconds and the combined PID differential to +/-100 us. Thus the I
contribution is at most `50*ki` us, and total correction is still limited to
100 us per motor around the 1300 us base. Watch `i` and `pid`: a persistent error
at the limit indicates insufficient correction authority, not a reason to keep
increasing gains. Use the course tuning procedure after fixing the hardware.

No guessed motor trim, higher output limit or automatic thrust calibration is
introduced. Record propeller markings, rotation directions, and angle/target/
P/I/D/PID/motor pulse telemetry if an imbalance remains.
