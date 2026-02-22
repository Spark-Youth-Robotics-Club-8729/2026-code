// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // Store setpoints so at-goal checks can compare against them properly
  private double leftFlywheelSetpointRPM = 0.0;
  private double rightFlywheelSetpointRPM = 0.0;
  private double hoodSetpointRad = 0.0;

  // Disconnection alerts
  private final Alert leftFlywheelDisconnected =
      new Alert("Left flywheel motor disconnected!", Alert.AlertType.kWarning);
  private final Alert rightFlywheelDisconnected =
      new Alert("Right flywheel motor disconnected!", Alert.AlertType.kWarning);
  private final Alert hoodDisconnected =
      new Alert("Hood motor disconnected!", Alert.AlertType.kWarning);
  private final Alert indexerDisconnected =
      new Alert("Indexer motor disconnected!", Alert.AlertType.kWarning);

  /** Creates a new Shooter subsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      stop();
    }

    // Hardware disconnection alerts
    leftFlywheelDisconnected.set(!inputs.leftFlywheelConnected);
    rightFlywheelDisconnected.set(!inputs.rightFlywheelConnected);
    hoodDisconnected.set(!inputs.hoodConnected);
    indexerDisconnected.set(!inputs.indexerConnected);

    // Log computed values
    Logger.recordOutput("Shooter/LeftFlywheelAtSpeed", isLeftFlywheelAtSpeed());
    Logger.recordOutput("Shooter/RightFlywheelAtSpeed", isRightFlywheelAtSpeed());
    Logger.recordOutput("Shooter/FlywheelsAtSpeed", areFlywheelsAtSpeed());
    Logger.recordOutput("Shooter/HoodAtPosition", isHoodAtPosition());
    Logger.recordOutput("Shooter/LeftFlywheelSetpointRPM", leftFlywheelSetpointRPM);
    Logger.recordOutput("Shooter/RightFlywheelSetpointRPM", rightFlywheelSetpointRPM);
    Logger.recordOutput("Shooter/HoodSetpointRad", hoodSetpointRad);
  }

  /**
   * Sets the velocity for both flywheels to the same speed.
   *
   * @param velocityRPM The target velocity in RPM.
   */
  public void setFlywheelVelocity(double velocityRPM) {
    leftFlywheelSetpointRPM = velocityRPM;
    rightFlywheelSetpointRPM = velocityRPM;
    io.setLeftFlywheelVelocity(velocityRPM);
    io.setRightFlywheelVelocity(velocityRPM);
  }

  /**
   * Sets different velocities for the left and right flywheels independently.
   *
   * @param leftVelocityRPM  Target velocity for the left flywheel in RPM.
   * @param rightVelocityRPM Target velocity for the right flywheel in RPM.
   */
  public void setFlywheelVelocities(double leftVelocityRPM, double rightVelocityRPM) {
    leftFlywheelSetpointRPM = leftVelocityRPM;
    rightFlywheelSetpointRPM = rightVelocityRPM;
    io.setLeftFlywheelVelocity(leftVelocityRPM);
    io.setRightFlywheelVelocity(rightVelocityRPM);
  }

  /**
   * Sets the hood angle.
   *
   * @param angleRad Target angle in radians (mechanism angle after gear reduction).
   */
  public void setHoodPosition(double angleRad) {
    hoodSetpointRad = angleRad;
    io.setHoodPosition(angleRad);
  }

  /** Runs the indexer counter-clockwise to feed a ball up to the shooter. */
  public void feedNote() {
    io.setIndexerVelocity(indexerFeedSpeedRPM);
    Logger.recordOutput("Shooter/IndexerState", "Feeding");
  }

  /** Runs the indexer clockwise to eject a ball back toward the hopper. */
  public void ejectNote() {
    io.setIndexerVelocity(indexerEjectSpeedRPM);
    Logger.recordOutput("Shooter/IndexerState", "Ejecting");
  }

  /** Stops the indexer. */
  public void stopIndexer() {
    io.setIndexerVelocity(0.0);
    Logger.recordOutput("Shooter/IndexerState", "Stopped");
  }

  /** Stops all shooter motors and clears setpoints. */
  public void stop() {
    leftFlywheelSetpointRPM = 0.0;
    rightFlywheelSetpointRPM = 0.0;
    io.stop();
    Logger.recordOutput("Shooter/IndexerState", "Stopped");
  }

  // ---------------------------------------------------------------------------
  // Getters
  // ---------------------------------------------------------------------------

  /** @return Current left flywheel velocity in RPM. */
  public double getLeftFlywheelVelocity() {
    return inputs.leftFlywheelVelocityRPM;
  }

  /** @return Current right flywheel velocity in RPM. */
  public double getRightFlywheelVelocity() {
    return inputs.rightFlywheelVelocityRPM;
  }

  /** @return Current hood mechanism angle in radians. */
  public double getHoodPosition() {
    return inputs.hoodPositionRad;
  }

  /** @return Average of left and right flywheel velocities in RPM. */
  public double getAverageFlywheelVelocity() {
    return (inputs.leftFlywheelVelocityRPM + inputs.rightFlywheelVelocityRPM) / 2.0;
  }

  // ---------------------------------------------------------------------------
  // At-goal checks  (compare measured value to stored setpoint)
  // ---------------------------------------------------------------------------

  /**
   * Returns true when the left flywheel is within {@code flywheelToleranceRPM} of its setpoint.
   */
  public boolean isLeftFlywheelAtSpeed() {
    return Math.abs(inputs.leftFlywheelVelocityRPM - leftFlywheelSetpointRPM)
        <= flywheelToleranceRPM;
  }

  /**
   * Returns true when the right flywheel is within {@code flywheelToleranceRPM} of its setpoint.
   */
  public boolean isRightFlywheelAtSpeed() {
    return Math.abs(inputs.rightFlywheelVelocityRPM - rightFlywheelSetpointRPM)
        <= flywheelToleranceRPM;
  }

  /** Returns true when both flywheels are at their setpoints. */
  public boolean areFlywheelsAtSpeed() {
    return isLeftFlywheelAtSpeed() && isRightFlywheelAtSpeed();
  }

  /**
   * Returns true when the hood is within {@code hoodToleranceRad} of its setpoint.
   * Compares measured position to stored setpoint (not velocity).
   */
  public boolean isHoodAtPosition() {
    return Math.abs(inputs.hoodPositionRad - hoodSetpointRad) <= hoodToleranceRad;
  }

  /** Returns true when the robot is ready to shoot (both flywheels at speed and hood at angle). */
  public boolean isReadyToShoot() {
    return areFlywheelsAtSpeed() && isHoodAtPosition();
  }
}