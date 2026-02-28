// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IndexerIOSparkMax implements IndexerIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;

  public IndexerIOSparkMax() {
    motor = new SparkMax(motorID, MotorType.kBrushless);
    encoder = motor.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(false); // TODO: flip to true if direction is wrong
    config.encoder
        // Convert so velocity reads in output shaft RPM (after 4:1 gearbox)
        .velocityConversionFactor(1.0 / gearRatio);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.connected = !motor.hasActiveFault();
    inputs.velocityRPM = encoder.getVelocity(); // already in output RPM via conversion factor
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {
    motor.setVoltage(outputs.volts);
  }
}
