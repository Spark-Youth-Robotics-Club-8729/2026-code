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

    // Hood pivot (uses built-in TalonFX encoder)
    public boolean hoodConnected = false;
    public double hoodPositionRad = 0.0;
    public double hoodVelocityRadPerSec = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
    public double hoodTempCelsius = 0.0;

    // Feeder (green wheels, feeds ball from hopper up to shooter — NOT the hopper indexer)
    public boolean feederConnected = false;
    public double feederVelocityRPM = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
    public double feederTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set the left flywheel velocity in RPM. */
  public default void setLeftFlywheelVelocity(double velocityRPM) {}

  /** Set the right flywheel velocity in RPM. */
  public default void setRightFlywheelVelocity(double velocityRPM) {}

  /** Set the hood position in radians (mechanism angle, after gear reduction). */
  public default void setHoodPosition(double positionRad) {}

  /** Set the feeder (green wheels) velocity in RPM. Positive = toward shooter, negative = eject. */
  public default void setFeederVelocity(double velocityRPM) {}

  /** Stop all shooter motors. */
  public default void stop() {}
}
