// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Hardware-agnostic shooter subsystem using AdvantageKit IO layer. */
public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double targetFlywheelRPM = 0.0;
  private double targetHoodAngleDeg = ShooterConstants.hoodMinAngleDeg;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/TargetFlywheelRPM", targetFlywheelRPM);
    Logger.recordOutput("Shooter/TargetHoodAngleDeg", targetHoodAngleDeg);
    Logger.recordOutput("Shooter/FlywheelAtSpeed", isFlywheelAtSpeed());
    Logger.recordOutput("Shooter/AverageFlywheelRPM", getAverageFlywheelRPM());
  }

  /** Sets both flywheels to the target velocity. */
  public void setFlywheelVelocity(double rpm) {
    targetFlywheelRPM = MathUtil.clamp(rpm, ShooterConstants.minFlywheelRPM, ShooterConstants.maxFlywheelRPM);
    io.setFlywheelVelocity(targetFlywheelRPM);
  }

  /** Sets each flywheel to a different velocity (for spin control). */
  public void setFlywheelVelocity(double leftRPM, double rightRPM) {
    io.setLeftFlywheelVelocity(MathUtil.clamp(leftRPM, ShooterConstants.minFlywheelRPM, ShooterConstants.maxFlywheelRPM));
    io.setRightFlywheelVelocity(MathUtil.clamp(rightRPM, ShooterConstants.minFlywheelRPM, ShooterConstants.maxFlywheelRPM));
    targetFlywheelRPM = (leftRPM + rightRPM) / 2.0;
  }

  /** Sets the hood to the target angle in degrees. */
  public void setHoodAngle(double angleDeg) {
    targetHoodAngleDeg = MathUtil.clamp(angleDeg, ShooterConstants.hoodMinAngleDeg, ShooterConstants.hoodMaxAngleDeg);
    io.setHoodPosition(targetHoodAngleDeg);
  }

  /** Runs the indexer at the specified velocity. */
  public void setIndexerVelocity(double rpm) {
    io.setIndexerVelocity(rpm);
  }

  /** Runs the indexer with open loop voltage. */
  public void setIndexerOpenLoop(double volts) {
    io.setIndexerOpenLoop(volts);
  }

  /** Stops the indexer. */
  public void stopIndexer() {
    io.setIndexerVelocity(0.0);
  }

  /** Stops all shooter motors. */
  public void stop() {
    targetFlywheelRPM = 0.0;
    io.stop();
  }

  /** Returns the average velocity of both flywheels in RPM. */
  public double getAverageFlywheelRPM() {
    return (inputs.leftFlywheelVelocityRPM + inputs.rightFlywheelVelocityRPM) / 2.0;
  }

  /** Returns the left flywheel velocity in RPM. */
  public double getLeftFlywheelRPM() {
    return inputs.leftFlywheelVelocityRPM;
  }

  /** Returns the right flywheel velocity in RPM. */
  public double getRightFlywheelRPM() {
    return inputs.rightFlywheelVelocityRPM;
  }

  /** Returns the current hood angle in degrees. */
  public double getHoodAngleDeg() {
    return inputs.hoodPositionDeg;
  }

  /** Returns the target flywheel RPM. */
  public double getTargetFlywheelRPM() {
    return targetFlywheelRPM;
  }

  /** Returns the error between target and actual flywheel RPM. */
  public double getFlywheelError() {
    return targetFlywheelRPM - getAverageFlywheelRPM();
  }

  /** Returns true if the flywheel is within tolerance of the target speed. */
  public boolean isFlywheelAtSpeed() {
    if (targetFlywheelRPM < 100) return false; // not spinning
    return Math.abs(getFlywheelError()) < ShooterConstants.flywheelToleranceRPM;
  }

  /** Returns true if both flywheels are connected. */
  public boolean isFlywheelConnected() {
    return inputs.leftFlywheelConnected && inputs.rightFlywheelConnected;
  }

  /** Returns true if the hood motor is connected. */
  public boolean isHoodConnected() {
    return inputs.hoodConnected;
  }

  /** Returns true if the indexer motor is connected. */
  public boolean isIndexerConnected() {
    return inputs.indexerConnected;
  }
}
