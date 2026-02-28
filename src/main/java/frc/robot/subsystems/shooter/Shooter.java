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
  // Initialize to min angle — matches where the encoder is seeded on startup
  private double hoodSetpointRad = hoodMinAngleRad;

  // Disconnection alerts
  private final Alert leftFlywheelDisconnected =
      new Alert("Left flywheel motor disconnected!", Alert.AlertType.kWarning);
  private final Alert rightFlywheelDisconnected =
      new Alert("Right flywheel motor disconnected!", Alert.AlertType.kWarning);
  private final Alert hoodDisconnected =
      new Alert("Hood motor disconnected!", Alert.AlertType.kWarning);
  private final Alert feederDisconnected =
      new Alert("Feeder motor disconnected!", Alert.AlertType.kWarning);

  /** Creates a new Shooter subsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Stop flywheels and feeder when disabled, but let hood hold position via io.stop() brake
    if (DriverStation.isDisabled()) {
      leftFlywheelSetpointRPM = 0.0;
      rightFlywheelSetpointRPM = 0.0;
      io.setLeftFlywheelVelocity(0.0);
      io.setRightFlywheelVelocity(0.0);
      io.setFeederVelocity(0.0);
      Logger.recordOutput("Shooter/FeederState", "Stopped");
    }

    // Hardware disconnection alerts
    leftFlywheelDisconnected.set(!inputs.leftFlywheelConnected);
    rightFlywheelDisconnected.set(!inputs.rightFlywheelConnected);
    hoodDisconnected.set(!inputs.hoodConnected);
    feederDisconnected.set(!inputs.feederConnected);

    // Log computed values
    Logger.recordOutput("Shooter/LeftFlywheelAtSpeed", isLeftFlywheelAtSpeed());
    Logger.recordOutput("Shooter/RightFlywheelAtSpeed", isRightFlywheelAtSpeed());
    Logger.recordOutput("Shooter/FlywheelsAtSpeed", areFlywheelsAtSpeed());
    Logger.recordOutput("Shooter/HoodAtPosition", isHoodAtPosition());
    Logger.recordOutput("Shooter/HoodPositionErrorRad", inputs.hoodPositionRad - hoodSetpointRad);
    Logger.recordOutput(
        "Shooter/HoodPositionErrorDeg", Math.toDegrees(inputs.hoodPositionRad - hoodSetpointRad));
    Logger.recordOutput("Shooter/LeftFlywheelSetpointRPM", leftFlywheelSetpointRPM);
    Logger.recordOutput("Shooter/RightFlywheelSetpointRPM", rightFlywheelSetpointRPM);
    Logger.recordOutput("Shooter/HoodSetpointRad", hoodSetpointRad);
    Logger.recordOutput("Shooter/IsReadyToShoot", isReadyToShoot());
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
   * @param leftVelocityRPM Target velocity for the left flywheel in RPM.
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

  /** Runs the feeder (green wheels) to feed a ball up to the shooter. */
  public void feedNote() {
    io.setFeederVelocity(feederFeedSpeedRPM);
    Logger.recordOutput("Shooter/FeederState", "Feeding");
  }

  /** Runs the feeder in reverse to eject a ball back toward the hopper. */
  public void ejectNote() {
    io.setFeederVelocity(feederEjectSpeedRPM);
    Logger.recordOutput("Shooter/FeederState", "Ejecting");
  }

  /** Stops the feeder. */
  public void stopFeeder() {
    io.setFeederVelocity(0.0);
    Logger.recordOutput("Shooter/FeederState", "Stopped");
  }

  /** Stops flywheel and hood motors and clears flywheel setpoints. Does NOT stop the feeder. */
  public void stop() {
    leftFlywheelSetpointRPM = 0.0;
    rightFlywheelSetpointRPM = 0.0;
    io.setLeftFlywheelVelocity(0.0);
    io.setRightFlywheelVelocity(0.0);
    io.setHoodPosition(hoodSetpointRad); // hold hood position, don't cut power
  }

  // ---------------------------------------------------------------------------
  // Getters
  // ---------------------------------------------------------------------------

  /**
   * @return Current left flywheel velocity in RPM.
   */
  public double getLeftFlywheelVelocity() {
    return inputs.leftFlywheelVelocityRPM;
  }

  /**
   * @return Current right flywheel velocity in RPM.
   */
  public double getRightFlywheelVelocity() {
    return inputs.rightFlywheelVelocityRPM;
  }

  /**
   * @return Current hood mechanism angle in radians.
   */
  public double getHoodPosition() {
    return inputs.hoodPositionRad;
  }

  /**
   * @return Average of left and right flywheel velocities in RPM.
   */
  public double getAverageFlywheelVelocity() {
    return (inputs.leftFlywheelVelocityRPM + inputs.rightFlywheelVelocityRPM) / 2.0;
  }

  // ---------------------------------------------------------------------------
  // At-goal checks  (compare measured value to stored setpoint)
  // ---------------------------------------------------------------------------

  /** Returns true when the left flywheel is within {@code flywheelToleranceRPM} of its setpoint. */
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
    return leftFlywheelSetpointRPM > 0.0 && isLeftFlywheelAtSpeed() && isRightFlywheelAtSpeed();
  }

  /**
   * Returns true when the hood is within {@code hoodToleranceRad} of its setpoint. Compares
   * measured position to stored setpoint (not velocity).
   */
  public boolean isHoodAtPosition() {
    return Math.abs(inputs.hoodPositionRad - hoodSetpointRad) <= hoodToleranceRad;
  }

  /** Returns true when the robot is ready to shoot (both flywheels at speed and hood at angle). */
  public boolean isReadyToShoot() {
    return areFlywheelsAtSpeed() && isHoodAtPosition();
  }

  /**
   * Convenience method used by AutoShootCommand — sets hood and both flywheels in one call.
   *
   * @param hoodAngleRad Target hood angle in radians.
   * @param flywheelSpeedRPM Target flywheel speed in RPM (applied to both wheels equally).
   */
  public void applyShootingParameters(double hoodAngleRad, double flywheelSpeedRPM) {
    setHoodPosition(hoodAngleRad);
    setFlywheelVelocity(flywheelSpeedRPM);
  }
}
