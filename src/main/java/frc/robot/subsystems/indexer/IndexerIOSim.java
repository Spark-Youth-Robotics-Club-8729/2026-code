// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private static final double dt = TimedRobot.kDefaultPeriod;

  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, gearRatio),
          DCMotor.getNEO(1));

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(dt);

    inputs.connected = true;
    inputs.velocityRPM = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelsius = 25.0;
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {
    appliedVolts = outputs.volts;
  }
}
