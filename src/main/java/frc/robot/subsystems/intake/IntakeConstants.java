// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

public class IntakeConstants {
  // CAN IDs
  public static final int rollerMotorID = 20;
  public static final int slapdownMotorID = 21;
  public static final String canBus = "rio"; // Change to "canivore" if needed

  // Slapdown gear ratio (motor rotations per mechanism rotation)
  public static final double slapdownGearRatio = 45; // changed gear ratio to 45:1

  // Slapdown angle limits — values from actual encoder readings on hardware.
  // NOTE: encoder reads ~0 at UP and ~0.0923 at DOWN (very small range — encoder
  // may not be fully coupled to output shaft; verify hardware before adjusting).
  public static final double slapdownUpAngleRad = 0.0;
  public static final double slapdownDownAngleRad = 0.0923;

  // Slapdown PID gains (down)
  public static final double slapdownDownKp = 0.08; // TODO: Tune
  public static final double slapdownDownKd = 0.4; // TODO: Tune

  // Slapdown PID gains (up)
  public static final double slapdownUpKp = 0.08; // TODO: Tune
  public static final double slapdownUpKd = 0.4; // TODO: Tune

  // Slapdown tolerance — must be less than the full encoder range (~0.0923 rad)
  public static final double slapdownToleranceRad = 0.02;

  // Roller voltages (positive = intake in, negative = outtake)
  public static final double rollerIntakeVolts = 6.0; // Reduced from 10.0 — TODO: Tune
  public static final double rollerOuttakeVolts = -4.0; // Reduced from -6.0 — TODO: Tune

  // Jitter constans
  public static final double jitterFrequencyHz = 0.3; // Tune
  public static final double jitterAmplitudeDeg = 70.0; // Tune

  private IntakeConstants() {}
}
