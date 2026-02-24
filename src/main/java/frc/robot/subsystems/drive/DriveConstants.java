// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(26.5);
  public static final double wheelBase = Units.inchesToMeters(26.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Chassis angular offsets for each module (from REV MAXSwerve template)
  public static final Rotation2d frontLeftZeroRotation = Rotation2d.fromRadians(-Math.PI / 2);
  public static final Rotation2d frontRightZeroRotation = Rotation2d.fromRadians(0);
  public static final Rotation2d backLeftZeroRotation = Rotation2d.fromRadians(Math.PI);
  public static final Rotation2d backRightZeroRotation = Rotation2d.fromRadians(Math.PI / 2);

  // Device CAN IDs
  public static final int frontLeftDriveCanId = 2;
  public static final int backLeftDriveCanId = 4;
  public static final int frontRightDriveCanId = 6;
  public static final int backRightDriveCanId = 8;

  public static final int frontLeftTurnCanId = 1;
  public static final int backLeftTurnCanId = 3;
  public static final int frontRightTurnCanId = 5;
  public static final int backRightTurnCanId = 7;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = 0.0762 / 2.0; // 3-inch wheel
  public static final int kDrivingMotorPinionTeeth = 12;
  public static final double driveMotorReduction =
      (45.0 * 22.0) / (kDrivingMotorPinionTeeth * 15.0); // 5.5 with 12T pinion
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration — output in meters and meters/sec
  public static final double driveEncoderPositionFactor =
      (wheelRadiusMeters * 2 * Math.PI) / driveMotorReduction; // Rotor Rotations -> Wheel Meters
  public static final double driveEncoderVelocityFactor =
      driveEncoderPositionFactor / 60.0; // Rotor RPM -> Wheel m/s

  // Drive PID/FF configuration (velocity control in m/s)
  // kV = nominalVoltage / freeSpeedMps — tells SparkMax how many volts per m/s
  public static final double driveKp = 0.04;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double kDrivingMotorFreeSpeedRps = 5676.0 / 60.0; // NEO free speed
  public static final double kDriveWheelFreeSpeedMps =
      (kDrivingMotorFreeSpeedRps * wheelRadiusMeters * 2 * Math.PI) / driveMotorReduction;
  public static final double driveKv = 12.0 / kDriveWheelFreeSpeedMps;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
