// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;

public class ShooterConstants {
  // CAN IDs - TODO: Update with actual IDs
  public static final int leftFlywheelID = 10;
  public static final int rightFlywheelID = 11;
  public static final int hoodMotorID = 12;
  public static final int indexerMotorID = 13;
  public static final String canBus = "rio";

  // Motor configuration
  public static final boolean leftFlywheelInverted = false;
  public static final boolean rightFlywheelInverted = true; // Opposite side
  public static final boolean hoodInverted = false;
  public static final boolean indexerInverted = false;

  // Current limits
  public static final int flywheelCurrentLimit = 40;
  public static final int hoodCurrentLimit = 30;
  public static final int indexerCurrentLimit = 30;

  // Flywheel configuration
  public static final double flywheelGearRatio = 1.0; // TODO: Set actual gear ratio
  public static final double flywheelMOI = 0.004; // Moment of inertia in kg*m^2
  public static final DCMotor flywheelGearbox = DCMotor.getKrakenX60(1);

  // Flywheel PID (for real hardware)
  public static final double flywheelKp = 0.1;
  public static final double flywheelKi = 0.0;
  public static final double flywheelKd = 0.0;
  public static final double flywheelKs = 0.0; // Static friction
  public static final double flywheelKv = 0.12; // Velocity feedforward (volts per RPS)

  // Flywheel PID (for simulation)
  public static final double flywheelSimKp = 0.5;
  public static final double flywheelSimKi = 0.0;
  public static final double flywheelSimKd = 0.0;
  public static final double flywheelSimKv = 0.12;

  // Hood configuration
  public static final double hoodGearRatio = 50.0; // TODO: Set actual gear ratio
  public static final double hoodMinAngleDeg = 20.0;
  public static final double hoodMaxAngleDeg = 60.0;
  public static final double hoodLengthMeters = 0.3; // For simulation arm model
  public static final double hoodMassKg = 1.0;
  public static final DCMotor hoodGearbox = DCMotor.getKrakenX60(1);

  // Hood PID
  public static final double hoodKp = 0.5;
  public static final double hoodKi = 0.0;
  public static final double hoodKd = 0.0;
  public static final double hoodSimKp = 2.0;
  public static final double hoodSimKd = 0.0;

  // Indexer configuration
  public static final double indexerGearRatio = 3.0; // TODO: Set actual gear ratio
  public static final double indexerMOI = 0.001;
  public static final DCMotor indexerGearbox = DCMotor.getKrakenX60(1);

  // Indexer PID
  public static final double indexerKp = 0.1;
  public static final double indexerKi = 0.0;
  public static final double indexerKd = 0.0;
  public static final double indexerKv = 0.12;

  // Physical limits
  public static final double minFlywheelRPM = 0.0;
  public static final double maxFlywheelRPM = 6000.0;
  public static final double flywheelToleranceRPM = 50.0;

  // Shooting physics
  public static final double wheelDiameterMeters = 0.1016; // 4 inches
  public static final double gravity = 9.8067; // m/s^2
  public static final double shooterHeightMeters = 0.75; // TODO: Measure actual height
  public static final double hubCenterHeightMeters = 2.36; // Target height
  public static final double heightDiffMeters = hubCenterHeightMeters - shooterHeightMeters;
  public static final double flightTimeSeconds = 0.5; // Experimentally determined
  public static final double defaultLaunchAngleDeg = 45.0;
}
