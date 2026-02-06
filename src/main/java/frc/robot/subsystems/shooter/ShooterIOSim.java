// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Simulation implementation for the shooter. */
public class ShooterIOSim implements ShooterIO {

  private final FlywheelSim leftFlywheelSim;
  private final FlywheelSim rightFlywheelSim;
  private final SingleJointedArmSim hoodSim;
  private final FlywheelSim indexerSim;

  private final PIDController leftFlywheelPID;
  private final PIDController rightFlywheelPID;
  private final PIDController hoodPID;
  private final PIDController indexerPID;

  private double leftFlywheelAppliedVolts = 0.0;
  private double rightFlywheelAppliedVolts = 0.0;
  private double hoodAppliedVolts = 0.0;
  private double indexerAppliedVolts = 0.0;

  private Double leftFlywheelSetpointRPM = null; // null means open loop
  private Double rightFlywheelSetpointRPM = null;
  private Double hoodSetpointDeg = null;
  private Double indexerSetpointRPM = null;

  public ShooterIOSim() {
    leftFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                ShooterConstants.flywheelMOI,
                ShooterConstants.flywheelGearRatio),
            DCMotor.getKrakenX60(1));

    rightFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                ShooterConstants.flywheelMOI,
                ShooterConstants.flywheelGearRatio),
            DCMotor.getKrakenX60(1));

    hoodSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            ShooterConstants.hoodGearRatio,
            SingleJointedArmSim.estimateMOI(
                ShooterConstants.hoodLengthMeters, ShooterConstants.hoodMassKg),
            ShooterConstants.hoodLengthMeters,
            Math.toRadians(ShooterConstants.hoodMinAngleDeg),
            Math.toRadians(ShooterConstants.hoodMaxAngleDeg),
            true,
            Math.toRadians(ShooterConstants.hoodMinAngleDeg));

    indexerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                ShooterConstants.indexerMOI,
                ShooterConstants.indexerGearRatio),
            DCMotor.getKrakenX60(1));

    leftFlywheelPID =
        new PIDController(
            ShooterConstants.flywheelSimKp,
            ShooterConstants.flywheelSimKi,
            ShooterConstants.flywheelSimKd);

    rightFlywheelPID =
        new PIDController(
            ShooterConstants.flywheelSimKp,
            ShooterConstants.flywheelSimKi,
            ShooterConstants.flywheelSimKd);

    hoodPID =
        new PIDController(
            ShooterConstants.hoodSimKp, 0.0, ShooterConstants.hoodSimKd);

    indexerPID =
        new PIDController(
            ShooterConstants.flywheelSimKp,
            ShooterConstants.flywheelSimKi,
            ShooterConstants.flywheelSimKd);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // run closed loop control if setpoints are set
    if (leftFlywheelSetpointRPM != null) {
      double currentRPM = leftFlywheelSim.getAngularVelocityRPM();
      double pidOutput = leftFlywheelPID.calculate(currentRPM, leftFlywheelSetpointRPM);
      double ffOutput = ShooterConstants.flywheelSimKv * leftFlywheelSetpointRPM / 60.0; // kv is per rps
      leftFlywheelAppliedVolts = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
    }

    if (rightFlywheelSetpointRPM != null) {
      double currentRPM = rightFlywheelSim.getAngularVelocityRPM();
      double pidOutput = rightFlywheelPID.calculate(currentRPM, rightFlywheelSetpointRPM);
      double ffOutput = ShooterConstants.flywheelSimKv * rightFlywheelSetpointRPM / 60.0;
      rightFlywheelAppliedVolts = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
    }

    if (hoodSetpointDeg != null) {
      double currentDeg = Math.toDegrees(hoodSim.getAngleRads());
      double pidOutput = hoodPID.calculate(currentDeg, hoodSetpointDeg);
      hoodAppliedVolts = MathUtil.clamp(pidOutput, -12.0, 12.0);
    }

    if (indexerSetpointRPM != null) {
      double currentRPM = indexerSim.getAngularVelocityRPM();
      double pidOutput = indexerPID.calculate(currentRPM, indexerSetpointRPM);
      double ffOutput = ShooterConstants.indexerKv * indexerSetpointRPM / 60.0;
      indexerAppliedVolts = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
    }

    // update physics simulation (20ms timestep)
    leftFlywheelSim.setInputVoltage(leftFlywheelAppliedVolts);
    leftFlywheelSim.update(0.02);

    rightFlywheelSim.setInputVoltage(rightFlywheelAppliedVolts);
    rightFlywheelSim.update(0.02);

    hoodSim.setInputVoltage(hoodAppliedVolts);
    hoodSim.update(0.02);

    indexerSim.setInputVoltage(indexerAppliedVolts);
    indexerSim.update(0.02);

    // populate inputs
    inputs.leftFlywheelConnected = true;
    inputs.leftFlywheelVelocityRPM = leftFlywheelSim.getAngularVelocityRPM();
    inputs.leftFlywheelAppliedVolts = leftFlywheelAppliedVolts;
    inputs.leftFlywheelCurrentAmps = leftFlywheelSim.getCurrentDrawAmps();
    inputs.leftFlywheelTempCelsius = 25.0; // simulated room temp

    inputs.rightFlywheelConnected = true;
    inputs.rightFlywheelVelocityRPM = rightFlywheelSim.getAngularVelocityRPM();
    inputs.rightFlywheelAppliedVolts = rightFlywheelAppliedVolts;
    inputs.rightFlywheelCurrentAmps = rightFlywheelSim.getCurrentDrawAmps();
    inputs.rightFlywheelTempCelsius = 25.0;

    inputs.hoodConnected = true;
    inputs.hoodPositionDeg = Math.toDegrees(hoodSim.getAngleRads());
    inputs.hoodVelocityDegPerSec = Math.toDegrees(hoodSim.getVelocityRadPerSec());
    inputs.hoodAppliedVolts = hoodAppliedVolts;
    inputs.hoodCurrentAmps = hoodSim.getCurrentDrawAmps();

    inputs.indexerConnected = true;
    inputs.indexerVelocityRPM = indexerSim.getAngularVelocityRPM();
    inputs.indexerAppliedVolts = indexerAppliedVolts;
    inputs.indexerCurrentAmps = indexerSim.getCurrentDrawAmps();
  }

  @Override
  public void setLeftFlywheelVelocity(double velocityRPM) {
    leftFlywheelSetpointRPM = velocityRPM;
  }

  @Override
  public void setRightFlywheelVelocity(double velocityRPM) {
    rightFlywheelSetpointRPM = velocityRPM;
  }

  @Override
  public void setFlywheelVelocity(double velocityRPM) {
    setLeftFlywheelVelocity(velocityRPM);
    setRightFlywheelVelocity(velocityRPM);
  }

  @Override
  public void setLeftFlywheelOpenLoop(double volts) {
    leftFlywheelSetpointRPM = null; // switch to open loop
    leftFlywheelAppliedVolts = volts;
  }

  @Override
  public void setRightFlywheelOpenLoop(double volts) {
    rightFlywheelSetpointRPM = null;
    rightFlywheelAppliedVolts = volts;
  }

  @Override
  public void setHoodPosition(double angleDeg) {
    hoodSetpointDeg = angleDeg;
  }

  @Override
  public void setHoodOpenLoop(double volts) {
    hoodSetpointDeg = null;
    hoodAppliedVolts = volts;
  }

  @Override
  public void setIndexerVelocity(double velocityRPM) {
    indexerSetpointRPM = velocityRPM;
  }

  @Override
  public void setIndexerOpenLoop(double volts) {
    indexerSetpointRPM = null;
    indexerAppliedVolts = volts;
  }

  @Override
  public void stop() {
    leftFlywheelSetpointRPM = null;
    rightFlywheelSetpointRPM = null;
    hoodSetpointDeg = null;
    indexerSetpointRPM = null;
    leftFlywheelAppliedVolts = 0.0;
    rightFlywheelAppliedVolts = 0.0;
    hoodAppliedVolts = 0.0;
    indexerAppliedVolts = 0.0;
  }
}
