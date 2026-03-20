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

  // Slapdown gear ratio (motor rotations per mechanism rotation)
  public static final double slapdownGearRatio = 45; // changed gear ratio to 45:1

  // Slapdown angle limits (radians, at the mechanism after gear reduction)
  public static final double slapdownUpAngleRad = Units.degreesToRadians(0.0); // TODO: Set actual
  public static final double slapdownDownAngleRad =
      Units.degreesToRadians(90.0); // TODO: Set actual

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