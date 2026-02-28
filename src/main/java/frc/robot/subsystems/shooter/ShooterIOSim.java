// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** IO implementation for shooter simulation using WPILib physics sim. */
public class ShooterIOSim implements ShooterIO {
  // ---------------------------------------------------------------------------
  // Flywheel sims
  // Combined MOI from ShooterConstants (converted from 16.436 in²·lb to kg·m²).
  // Each motor drives one wheel; the total MOI is shared across both since they
  // are coupled through the game piece. We split it 50/50 per wheel.
  // ---------------------------------------------------------------------------
  private final FlywheelSim leftFlywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1), flywheelMOI / 2.0, flywheelGearRatio),
          DCMotor.getKrakenX60(1));

  private final FlywheelSim rightFlywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1), flywheelMOI / 2.0, flywheelGearRatio),
          DCMotor.getKrakenX60(1));

  // ---------------------------------------------------------------------------
  // Hood sim — SingleJointedArmSim, gear ratio applied here
  // ---------------------------------------------------------------------------
  private final SingleJointedArmSim hoodSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          hoodGearRatio, // 13.7778:1
          0.02, // MOI in kg·m² (estimate for the hood pivot arm)
          0.3, // Arm length in meters (estimate)
          hoodMinAngleRad,
          hoodMaxAngleRad,
          true, // Simulate gravity
          (hoodMinAngleRad + hoodMaxAngleRad) / 2.0); // Start at middle

  // ---------------------------------------------------------------------------
  // Feeder sim (green wheels)
  // ---------------------------------------------------------------------------
  private final FlywheelSim feederSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.001, feederGearRatio),
          DCMotor.getKrakenX60(1));

  // ---------------------------------------------------------------------------
  // PID controllers (use same gains as hardware for consistency)
  // ---------------------------------------------------------------------------
  private final PIDController leftFlywheelPID =
      new PIDController(flywheelKp, flywheelKi, flywheelKd);
  private final PIDController rightFlywheelPID =
      new PIDController(flywheelKp, flywheelKi, flywheelKd);
  private final PIDController hoodPID = new PIDController(hoodKp, hoodKi, hoodKd);
  private final PIDController feederPID = new PIDController(feederKp, feederKi, feederKd);

  // Setpoints
  private double leftFlywheelSetpointRPM = 0.0;
  private double rightFlywheelSetpointRPM = 0.0;
  private double hoodSetpointRad = (hoodMinAngleRad + hoodMaxAngleRad) / 2.0;
  private double feederSetpointRPM = 0.0;

  // Applied voltages
  private double leftFlywheelAppliedVolts = 0.0;
  private double rightFlywheelAppliedVolts = 0.0;
  private double hoodAppliedVolts = 0.0;
  private double feederAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Use Constants.loopPeriodSecs so sim stays in sync with actual loop timing
    final double dt = edu.wpi.first.wpilibj.TimedRobot.kDefaultPeriod;

    // Left flywheel
    leftFlywheelAppliedVolts =
        MathUtil.clamp(
            leftFlywheelPID.calculate(
                    leftFlywheelSim.getAngularVelocityRPM(), leftFlywheelSetpointRPM)
                + flywheelKv * leftFlywheelSetpointRPM / 60.0,
            -12.0,
            12.0);
    leftFlywheelSim.setInputVoltage(leftFlywheelAppliedVolts);
    leftFlywheelSim.update(dt);

    // Right flywheel
    rightFlywheelAppliedVolts =
        MathUtil.clamp(
            rightFlywheelPID.calculate(
                    rightFlywheelSim.getAngularVelocityRPM(), rightFlywheelSetpointRPM)
                + flywheelKv * rightFlywheelSetpointRPM / 60.0,
            -12.0,
            12.0);
    rightFlywheelSim.setInputVoltage(rightFlywheelAppliedVolts);
    rightFlywheelSim.update(dt);

    // Hood
    hoodAppliedVolts =
        MathUtil.clamp(hoodPID.calculate(hoodSim.getAngleRads(), hoodSetpointRad), -12.0, 12.0);
    hoodSim.setInputVoltage(hoodAppliedVolts);
    hoodSim.update(dt);

    // Feeder
    feederAppliedVolts =
        MathUtil.clamp(
            feederPID.calculate(feederSim.getAngularVelocityRPM(), feederSetpointRPM)
                + feederKv * feederSetpointRPM / 60.0,
            -12.0,
            12.0);
    feederSim.setInputVoltage(feederAppliedVolts);
    feederSim.update(dt);

    // Write back to inputs
    inputs.leftFlywheelConnected = true;
    inputs.leftFlywheelVelocityRPM = leftFlywheelSim.getAngularVelocityRPM();
    inputs.leftFlywheelAppliedVolts = leftFlywheelAppliedVolts;
    inputs.leftFlywheelCurrentAmps = leftFlywheelSim.getCurrentDrawAmps();
    inputs.leftFlywheelTempCelsius = 25.0;

    inputs.rightFlywheelConnected = true;
    inputs.rightFlywheelVelocityRPM = rightFlywheelSim.getAngularVelocityRPM();
    inputs.rightFlywheelAppliedVolts = rightFlywheelAppliedVolts;
    inputs.rightFlywheelCurrentAmps = rightFlywheelSim.getCurrentDrawAmps();
    inputs.rightFlywheelTempCelsius = 25.0;

    inputs.hoodConnected = true;
    inputs.hoodPositionRad = hoodSim.getAngleRads();
    inputs.hoodVelocityRadPerSec = hoodSim.getVelocityRadPerSec();
    inputs.hoodAppliedVolts = hoodAppliedVolts;
    inputs.hoodCurrentAmps = hoodSim.getCurrentDrawAmps();
    inputs.hoodTempCelsius = 25.0;

    inputs.feederConnected = true;
    inputs.feederVelocityRPM = feederSim.getAngularVelocityRPM();
    inputs.feederAppliedVolts = feederAppliedVolts;
    inputs.feederCurrentAmps = feederSim.getCurrentDrawAmps();
    inputs.feederTempCelsius = 25.0;
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
  public void setHoodPosition(double positionRad) {
    hoodSetpointRad = MathUtil.clamp(positionRad, hoodMinAngleRad, hoodMaxAngleRad);
  }

  @Override
  public void setFeederVelocity(double velocityRPM) {
    feederSetpointRPM = velocityRPM;
  }

  @Override
  public void stop() {
    leftFlywheelSetpointRPM = 0.0;
    rightFlywheelSetpointRPM = 0.0;
    feederSetpointRPM = 0.0;
    // Hold hood at current position rather than letting it fall
    hoodSetpointRad = hoodSim.getAngleRads();
  }
}