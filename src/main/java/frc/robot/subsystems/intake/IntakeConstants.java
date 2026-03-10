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
  public static final String canBus = "rio"; 

  // Slapdown gear ratio (motor rotations per mechanism rotation)
  public static final double slapdownGearRatio = 270.0; // 9 x 5 x 3 x 2

  // Slapdown angle limits (radians, at the mechanism after gear reduction)
  public static final double slapdownUpAngleRad = Units.degreesToRadians(0.0); 
  public static final double slapdownDownAngleRad =
      Units.degreesToRadians(90.0); 

  // Slapdown PID gains
  public static final double slapdownKp = 0.5; // TODO: Tune
  public static final double slapdownKd = 0.5; // TODO: Tune

  // Slapdown tolerance
  public static final double slapdownToleranceRad = Units.degreesToRadians(2.0);

  // Roller voltages (positive = intake in, negative = outtake)
  public static final double rollerIntakeVolts = 6.0; // Reduced from 10.0 — TODO: Tune
  public static final double rollerOuttakeVolts = -4.0; // Reduced from -6.0 — TODO: Tune

  // Jitter parameters
  public static final double jitterAmplitudeRad = Units.degreesToRadians(12.0); // TODO: Tune
  public static final double jitterFrequencyHz = 8.0; // TODO: Tune

  // Absolute encoder offset (radians) — set this to the raw encoder reading when arm is at UP position
  public static final double slapdownAbsoluteEncoderOffsetRad = 0.0; // TODO: Measure and set

  // Stall detection
  public static final double slapdownStallCurrentAmps = 25.0; // TODO: Tune — current threshold to detect stall
  public static final double slapdownStallDebounceSeconds = 0.1; // How long current must exceed threshold


  private IntakeConstants() {}
}
