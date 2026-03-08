// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  // CAN IDs
  public static final int leftFlywheelID = 10; // TODO: Set actual CAN ID
  public static final int rightFlywheelID = 11; // TODO: Set actual CAN ID
  public static final int hoodMotorID = 12; // TODO: Set actual CAN ID
  public static final int feederMotorID = 13; // renamed from indexerMotorID — green feeder wheels
  public static final String canBus = "rio"; // Change to "canivore" if using CANivore

  // ---------------------------------------------------------------------------
  // Gear ratios
  // ---------------------------------------------------------------------------
  // Flywheel: 1:1 direct drive
  public static final double flywheelGearRatio = 1.0;

  // Hood pivot: 13.7778:1  (13.7778 motor rotations = 1 degree of hood travel)
  // TalonFX SensorToMechanismRatio expects motor-rotations / mechanism-rotations.
  // 13.7778 motor degrees per hood degree = 13.7778 motor rotations per hood rotation.
  public static final double hoodGearRatio = 13.7778;

  // Feeder (green wheels, hopper→shooter): 1:1.375  (1 motor rotation = 1.375 mechanism rotations)
  // Stored as motor-rotations / mechanism-rotations for SensorToMechanismRatio.
  public static final double feederGearRatio = 1.0 / 1.375; // ≈ 0.7273

  // ---------------------------------------------------------------------------
  // Flywheel physical constants
  // ---------------------------------------------------------------------------
  // Combined MOI of flywheel assembly (flywheel, shooter wheels, shaft, pulleys)
  // 16.436 in²·lb  →  kg·m²
  public static final double flywheelMOI =
      Units.lbsToKilograms(16.436 * Units.inchesToMeters(1.0) * Units.inchesToMeters(1.0));
  // Equivalent: 16.436 * 0.000292639 ≈ 0.004809 kg·m²

  // Radius of the flywheel wheel that contacts the note (used for physics-based RPM calculation).
  // TODO: Set to the actual shooter wheel radius.
  public static final double flywheelWheelRadiusM = Units.inchesToMeters(2.0);

  // Height of the shooter exit point above the floor (used for projectile Δh calculation).
  // TODO: Measure on the actual robot.
  public static final double shooterExitHeightM = Units.inchesToMeters(24.0);

  // Fraction of flywheel surface speed transferred to the note exit speed.
  // Accounts for wheel compression / slip (typical range: 0.80 – 0.95).
  // TODO: Characterise on the real robot (shoot at known distance, measure exit speed vs RPM).
  public static final double noteExitEfficiency = 0.85;

  // ---------------------------------------------------------------------------
  // Flywheel PID / feedforward constants
  // ---------------------------------------------------------------------------
  public static final double flywheelKp = 0.4;
  public static final double flywheelKi = 0.0;
  public static final double flywheelKd = 0.0;
  public static final double flywheelKv = 0.12; // Volts per RPS
  public static final double flywheelKs = 0.25; // Static friction — helps hold speed under load

  // ---------------------------------------------------------------------------
  // Hood PID constants
  // ---------------------------------------------------------------------------
  public static final double hoodKp = 24.0; // Increased — 1.0 may be too weak against gravity/load
  public static final double hoodKi = 0.0;
  public static final double hoodKd = 0.3; // Small D to damp oscillation

  // ---------------------------------------------------------------------------
  // Feeder (green wheels, hopper→shooter) PID / feedforward constants
  // ---------------------------------------------------------------------------
  public static final double feederKp = 0.1; // TODO: Tune
  public static final double feederKi = 0.0;
  public static final double feederKd = 0.0;
  public static final double feederKv = 0.12;

  // ---------------------------------------------------------------------------
  // Hood angle limits (radians, at the mechanism — after gear reduction)
  // ---------------------------------------------------------------------------
  public static final double hoodMinAngleRad = Units.degreesToRadians(10.0); // TODO: Set actual
  public static final double hoodMaxAngleRad = Units.degreesToRadians(60.0); // TODO: Set actual

  // ---------------------------------------------------------------------------
  // Tolerances
  // ---------------------------------------------------------------------------
  public static final double flywheelToleranceRPM = 100.8729; // widened 50 to 187.29
  public static final double hoodToleranceRad =
      Units.degreesToRadians(5.0); // Widened from 3° to 5° while tuning

  // ---------------------------------------------------------------------------
  // Default / feeder speeds
  // ---------------------------------------------------------------------------
  public static final double defaultFlywheelSpeedRPM = 3000.0; // TODO: Tune
  // Positive = feeds ball up from hopper to shooter
  public static final double feederFeedSpeedRPM = 500.0;
  // Negative = ejects ball back down toward hopper
  public static final double feederEjectSpeedRPM = -300.0;

  private ShooterConstants() {}
}
