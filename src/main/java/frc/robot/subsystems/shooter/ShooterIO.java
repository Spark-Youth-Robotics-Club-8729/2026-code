// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    // Left flywheel
    public boolean leftFlywheelConnected = false;
    public double leftFlywheelVelocityRPM = 0.0;
    public double leftFlywheelAppliedVolts = 0.0;
    public double leftFlywheelCurrentAmps = 0.0;
    public double leftFlywheelTempCelsius = 0.0;

    // Right flywheel
    public boolean rightFlywheelConnected = false;
    public double rightFlywheelVelocityRPM = 0.0;
    public double rightFlywheelAppliedVolts = 0.0;
    public double rightFlywheelCurrentAmps = 0.0;
    public double rightFlywheelTempCelsius = 0.0;

    // Hood
    public boolean hoodConnected = false;
    public double hoodPositionDeg = 0.0;
    public double hoodVelocityDegPerSec = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;

    // Indexer
    public boolean indexerConnected = false;
    public double indexerVelocityRPM = 0.0;
    public double indexerAppliedVolts = 0.0;
    public double indexerCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the left flywheel at the specified velocity in RPM. */
  public default void setLeftFlywheelVelocity(double velocityRPM) {}

  /** Run the right flywheel at the specified velocity in RPM. */
  public default void setRightFlywheelVelocity(double velocityRPM) {}

  /** Run both flywheels at the specified velocity in RPM. */
  public default void setFlywheelVelocity(double velocityRPM) {}

  /** Run the left flywheel at the specified voltage (open loop). */
  public default void setLeftFlywheelOpenLoop(double volts) {}

  /** Run the right flywheel at the specified voltage (open loop). */
  public default void setRightFlywheelOpenLoop(double volts) {}

  /** Set the hood to the specified angle in degrees. */
  public default void setHoodPosition(double angleDeg) {}

  /** Run the hood at the specified voltage (open loop). */
  public default void setHoodOpenLoop(double volts) {}

  /** Run the indexer at the specified velocity in RPM. */
  public default void setIndexerVelocity(double velocityRPM) {}

  /** Run the indexer at the specified voltage (open loop). */
  public default void setIndexerOpenLoop(double volts) {}

  /** Stop all motors. */
  public default void stop() {}
}
