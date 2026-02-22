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
  public static final int indexerMotorID = 13; // TODO: Set actual CAN ID
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

  // Indexer: 1:1.375  (1 motor rotation = 1.375 mechanism rotations)
  // Stored as motor-rotations / mechanism-rotations for SensorToMechanismRatio.
  public static final double indexerGearRatio = 1.0 / 1.375; // ≈ 0.7273

  // ---------------------------------------------------------------------------
  // Flywheel physical constants
  // ---------------------------------------------------------------------------
  // Combined MOI of flywheel assembly (flywheel, shooter wheels, shaft, pulleys)
  // 16.436 in²·lb  →  kg·m²
  public static final double flywheelMOI =
      Units.lbsToKilograms(16.436 * Units.inchesToMeters(1.0) * Units.inchesToMeters(1.0));
  // Equivalent: 16.436 * 0.000292639 ≈ 0.004809 kg·m²

  // ---------------------------------------------------------------------------
  // Flywheel PID / feedforward constants
  // ---------------------------------------------------------------------------
  public static final double flywheelKp = 0.1; // TODO: Tune
  public static final double flywheelKi = 0.0;
  public static final double flywheelKd = 0.0;
  public static final double flywheelKv = 0.12; // Volts per RPS feedforward

  // ---------------------------------------------------------------------------
  // Hood PID constants
  // ---------------------------------------------------------------------------
  public static final double hoodKp = 10.0; // TODO: Tune
  public static final double hoodKi = 0.0;
  public static final double hoodKd = 0.5;

  // ---------------------------------------------------------------------------
  // Indexer PID / feedforward constants
  // ---------------------------------------------------------------------------
  public static final double indexerKp = 0.1; // TODO: Tune
  public static final double indexerKi = 0.0;
  public static final double indexerKd = 0.0;
  public static final double indexerKv = 0.12;

  // ---------------------------------------------------------------------------
  // Hood angle limits (radians, at the mechanism — after gear reduction)
  // ---------------------------------------------------------------------------
  public static final double hoodMinAngleRad = Units.degreesToRadians(10.0); // TODO: Set actual
  public static final double hoodMaxAngleRad = Units.degreesToRadians(60.0); // TODO: Set actual

  // ---------------------------------------------------------------------------
  // Tolerances
  // ---------------------------------------------------------------------------
  public static final double flywheelToleranceRPM = 50.0;
  public static final double hoodToleranceRad = Units.degreesToRadians(1.0);

  // ---------------------------------------------------------------------------
  // Default / indexer speeds
  // ---------------------------------------------------------------------------
  public static final double defaultFlywheelSpeedRPM = 3000.0; // TODO: Tune
  // Positive = counter-clockwise (brings ball up from hopper to shooter)
  public static final double indexerFeedSpeedRPM = 500.0;
  // Negative = clockwise (ejects ball back down)
  public static final double indexerEjectSpeedRPM = -300.0;

  private ShooterConstants() {}
}
