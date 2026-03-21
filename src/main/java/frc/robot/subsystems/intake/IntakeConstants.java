// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {
  // CAN IDs
  public static final int rollerMotorID = 20;
  public static final int slapdownMotorID = 21;
  public static final String canBus = "rio"; // Change to "canivore" if needed

  // Zero offset (rotations) fed into SparkAbsoluteEncoder config.
  // Shifted 0.1 rot back from the raw UP reading (0.4135552) so that the
  // adjusted UP position is ~0.1 rot instead of ~0.0 rot, keeping both
  // endpoints away from the 0/1 wrap boundary and preventing the encoder
  // from bouncing between 0.000 and 0.998 at the UP position.
  public static final double slapdownEncoderOffset = 0.4135552 - 0.1; // = 0.3135552

  // Slapdown gear ratio (motor rotations per mechanism rotation)
  public static final double slapdownGearRatio = 45; // changed gear ratio to 45:1

  // Slapdown angle limits — expressed in radians from the absolute encoder.
  // UP  = 0.100 rot * 2π ≈ 0.628 rad
  // DOWN = (0.100 + 0.711) rot * 2π = 0.811 rot * 2π ≈ 5.095 rad
  public static final double slapdownUpAngleRad = 0.1 * 2.0 * Math.PI;
  public static final double slapdownDownAngleRad = 0.811 * 2.0 * Math.PI;

  // Slapdown PID gains (separate for each mode so they can be tuned independently)
  public static final double slapdownUpKp = 0.3; // TODO: Tune
  public static final double slapdownUpKd = 0.4; // TODO: Tune

  public static final double slapdownDownKp = 0.15; // TODO: Tune
  public static final double slapdownDownKd = 0.8; // TODO: Tune

  public static final double slapdownJitterKp = 0.5; // TODO: Tune
  public static final double slapdownJitterKd = 0.5; // TODO: Tune

  // Slapdown tolerance
  public static final double slapdownToleranceRad = Units.degreesToRadians(5.0);

  // Slapdown stall detection (stop PID if we likely hit the bumper)
  public static final double slapdownStallCurrentAmps = 25.0; // above normal running current
  public static final double slapdownStallAppliedVolts = 8.0; // near full output
  public static final double slapdownStallVelocityRadPerSec = Units.degreesToRadians(5.0);
  public static final double slapdownStallDebounceSec = 0.15;

  // Roller voltages (positive = intake in, negative = outtake)
  public static final double rollerIntakeVolts = 6.0; // Reduced from 10.0 — TODO: Tune
  public static final double rollerOuttakeVolts = -4.0; // Reduced from -6.0 — TODO: Tune

  // Jitter constans
  public static final double jitterFrequencyHz = 0.25; // Tune
  public static final double jitterAmplitudeDeg = 45.0; // Tune

  private IntakeConstants() {}
}
