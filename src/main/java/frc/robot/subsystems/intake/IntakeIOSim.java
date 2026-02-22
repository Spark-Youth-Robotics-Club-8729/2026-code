// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private static final double dt = TimedRobot.kDefaultPeriod;

  // Roller — Kraken X60, low inertia flywheel
  private final DCMotorSim rollerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1.0),
          DCMotor.getKrakenX60(1));

  // Slapdown — NEO brushless via SparkMax, modeled as arm so gravity works
  private final SingleJointedArmSim slapdownSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          slapdownGearRatio,
          0.05, // MOI estimate (kg·m²) — TODO: update with real value
          0.4, // Arm length estimate (m) — TODO: update with real value
          slapdownUpAngleRad,
          slapdownDownAngleRad,
          true, // Simulate gravity
          slapdownUpAngleRad); // Start retracted

  private final PIDController slapdownPID = new PIDController(slapdownKp, 0.0, slapdownKd);

  private double rollerAppliedVolts = 0.0;
  private double slapdownAppliedVolts = 0.0;
  private double slapdownGoalRad = slapdownUpAngleRad;
  private IntakeIOOutputMode slapdownMode = IntakeIOOutputMode.BRAKE;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Roller
    rollerSim.setInputVoltage(MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0));
    rollerSim.update(dt);

    // Slapdown
    switch (slapdownMode) {
      case BRAKE, COAST -> slapdownAppliedVolts = 0.0;
      case OPEN_LOOP -> {
        /* already set in applyOutputs */
      }
      case CLOSED_LOOP -> slapdownAppliedVolts =
          MathUtil.clamp(
              slapdownPID.calculate(slapdownSim.getAngleRads(), slapdownGoalRad), -12.0, 12.0);
    }
    slapdownSim.setInputVoltage(MathUtil.clamp(slapdownAppliedVolts, -12.0, 12.0));
    slapdownSim.update(dt);

    inputs.rollerConnected = true;
    inputs.rollerVelocityRPM = rollerSim.getAngularVelocityRPM();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerCurrentAmps = rollerSim.getCurrentDrawAmps();
    inputs.rollerTempCelsius = 25.0;

    inputs.slapdownConnected = true;
    inputs.slapdownPositionRad = slapdownSim.getAngleRads();
    inputs.slapdownVelocityRadPerSec = slapdownSim.getVelocityRadPerSec();
    inputs.slapdownAppliedVolts = slapdownAppliedVolts;
    inputs.slapdownCurrentAmps = slapdownSim.getCurrentDrawAmps();
    inputs.slapdownTempCelsius = 25.0;
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    slapdownMode = outputs.slapdownMode;
    rollerAppliedVolts = outputs.rollerVolts;

    // Update PID gains if they changed
    slapdownPID.setP(outputs.kP);
    slapdownPID.setD(outputs.kD);

    switch (outputs.slapdownMode) {
      case OPEN_LOOP -> slapdownAppliedVolts = outputs.slapdownVolts;
      case CLOSED_LOOP -> slapdownGoalRad = outputs.slapdownPositionRad;
      default -> {}
    }
  }
}
