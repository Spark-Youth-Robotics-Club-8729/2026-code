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
  public static final int leftFlywheelID = 10;
  public static final int rightFlywheelID = 11;
  public static final int hoodMotorID = 12;
  public static final int feederMotorID = 13; 
  public static final String canBus = "rio"; 

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
  // 16.436 in²·lb  →  kg·m²    // TODO: Is this even correct?
  public static final double flywheelMOI =
      Units.lbsToKilograms(16.436 * Units.inchesToMeters(1.0) * Units.inchesToMeters(1.0));
  // Equivalent: 16.436 * 0.000292639 ≈ 0.004809 kg·m²

  // Radius of the flywheel wheel that contacts the note (used for physics-based RPM calculation).
  public static final double flywheelWheelRadiusM = Units.inchesToMeters(4.0);

  // Height of the shooter exit point above the floor (used for projectile Δh calculation).
  public static final double shooterExitHeightM =
      Units.inchesToMeters(
          20.500); // TODO: CHANGED FROM 24.0 TO 20.5 (actual). This might mess up shooter
  // calculations though.

  // Fraction of flywheel surface speed transferred to the note exit speed.
  // Accounts for wheel compression / slip (typical range: 0.80 – 0.95).

  // ---------------------------------------------------------------------------
  // Flywheel PID / feedforward constants
  // ---------------------------------------------------------------------------
  public static final double flywheelKp = 0.4;   // TODO: Tune
  public static final double flywheelKi = 0.0;   // TODO: Tune
  public static final double flywheelKd = 0.0;   // TODO: Tune
  public static final double flywheelKv = 0.12; // Volts per RPS
  public static final double flywheelKs = 0.25; // Static friction — helps hold speed under load

  // ---------------------------------------------------------------------------
  // Hood PID constants
  // ---------------------------------------------------------------------------
  public static final double hoodKp = 24.0;   // TODO: Tune
  public static final double hoodKi = 0.0;   // TODO: Tune
  public static final double hoodKd = 0.3;   // TODO: Tune

  // ---------------------------------------------------------------------------
  // Feeder (green wheels, hopper→shooter) PID / feedforward constants
  // ---------------------------------------------------------------------------
  public static final double feederKp = 0.1; // TODO: Tune
  public static final double feederKi = 0.0;   // TODO: Tune
  public static final double feederKd = 0.0;   // TODO: Tune
  public static final double feederKv = 0.12;   // TODO: Tune

  // ---------------------------------------------------------------------------
  // Hood angle limits (radians, at the mechanism — after gear reduction)
  // ---------------------------------------------------------------------------
  public static final double hoodMinAngleRad = Units.degreesToRadians(5.0);
  public static final double hoodMaxAngleRad = Units.degreesToRadians(45.0);

  // ---------------------------------------------------------------------------
  // Tolerances
  // ---------------------------------------------------------------------------
  public static final double flywheelToleranceRPM = 100; // widened 50 to 100 RPM while tuning
  public static final double hoodToleranceRad =
      Units.degreesToRadians(5.0); // Widened from 3° to 5° while tuning

  // ---------------------------------------------------------------------------
  // Default / feeder speeds
  // ---------------------------------------------------------------------------
  public static final double defaultFlywheelSpeedRPM = 3000.0;
  public static final double maxFlywheelSpeedRPM = 6000.0;
  // Positive = feeds ball up from hopper to shooter
  public static final double feederFeedSpeedRPM = 800.0;
  // Negative = ejects ball back down toward hopper
  public static final double feederEjectSpeedRPM = -500.0;

  private ShooterConstants() {}
}
