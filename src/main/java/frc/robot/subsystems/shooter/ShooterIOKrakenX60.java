// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Real hardware implementation using Kraken X60 motors (TalonFX). */
public class ShooterIOKrakenX60 implements ShooterIO {

  private final TalonFX leftFlywheelMotor;
  private final TalonFX rightFlywheelMotor;
  private final TalonFX hoodMotor;
  private final TalonFX indexerMotor;

  private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0).withSlot(0); // reused each cycle
  private final VoltageOut flywheelVoltageRequest = new VoltageOut(0);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0).withSlot(0);
  private final VoltageOut hoodVoltageRequest = new VoltageOut(0);
  private final VelocityVoltage indexerVelocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut indexerVoltageRequest = new VoltageOut(0);

  private final StatusSignal<AngularVelocity> leftFlywheelVelocity;
  private final StatusSignal<Voltage> leftFlywheelAppliedVolts;
  private final StatusSignal<Current> leftFlywheelCurrent;
  private final StatusSignal<Temperature> leftFlywheelTemp;

  private final StatusSignal<AngularVelocity> rightFlywheelVelocity;
  private final StatusSignal<Voltage> rightFlywheelAppliedVolts;
  private final StatusSignal<Current> rightFlywheelCurrent;
  private final StatusSignal<Temperature> rightFlywheelTemp;

  private final StatusSignal<edu.wpi.first.units.measure.Angle> hoodPosition;
  private final StatusSignal<AngularVelocity> hoodVelocity;
  private final StatusSignal<Voltage> hoodAppliedVolts;
  private final StatusSignal<Current> hoodCurrent;

  private final StatusSignal<AngularVelocity> indexerVelocity;
  private final StatusSignal<Voltage> indexerAppliedVolts;
  private final StatusSignal<Current> indexerCurrent;

  public ShooterIOKrakenX60() {
    CANBus canBusInstance =
        ShooterConstants.canBus.equals("rio")
            ? new CANBus("rio")
            : new CANBus(ShooterConstants.canBus);

    leftFlywheelMotor = new TalonFX(ShooterConstants.leftFlywheelID, canBusInstance);
    rightFlywheelMotor = new TalonFX(ShooterConstants.rightFlywheelID, canBusInstance);
    hoodMotor = new TalonFX(ShooterConstants.hoodMotorID, canBusInstance);
    indexerMotor = new TalonFX(ShooterConstants.indexerMotorID, canBusInstance);

    var leftFlywheelConfig = new TalonFXConfiguration();
    leftFlywheelConfig.MotorOutput.Inverted =
        ShooterConstants.leftFlywheelInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    leftFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftFlywheelConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.flywheelCurrentLimit;
    leftFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftFlywheelConfig.Slot0.kP = ShooterConstants.flywheelKp;
    leftFlywheelConfig.Slot0.kI = ShooterConstants.flywheelKi;
    leftFlywheelConfig.Slot0.kD = ShooterConstants.flywheelKd;
    leftFlywheelConfig.Slot0.kS = ShooterConstants.flywheelKs;
    leftFlywheelConfig.Slot0.kV = ShooterConstants.flywheelKv;
    leftFlywheelMotor.getConfigurator().apply(leftFlywheelConfig);

    var rightFlywheelConfig = new TalonFXConfiguration();
    rightFlywheelConfig.MotorOutput.Inverted =
        ShooterConstants.rightFlywheelInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive; // inverted since motors face opposite directions
    rightFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightFlywheelConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.flywheelCurrentLimit;
    rightFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightFlywheelConfig.Slot0.kP = ShooterConstants.flywheelKp;
    rightFlywheelConfig.Slot0.kI = ShooterConstants.flywheelKi;
    rightFlywheelConfig.Slot0.kD = ShooterConstants.flywheelKd;
    rightFlywheelConfig.Slot0.kS = ShooterConstants.flywheelKs;
    rightFlywheelConfig.Slot0.kV = ShooterConstants.flywheelKv;
    rightFlywheelMotor.getConfigurator().apply(rightFlywheelConfig);

    var hoodConfig = new TalonFXConfiguration();
    hoodConfig.MotorOutput.Inverted =
        ShooterConstants.hoodInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // brake to hold position
    hoodConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.hoodCurrentLimit;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.Slot0.kP = ShooterConstants.hoodKp;
    hoodConfig.Slot0.kI = ShooterConstants.hoodKi;
    hoodConfig.Slot0.kD = ShooterConstants.hoodKd;
    hoodConfig.Feedback.SensorToMechanismRatio = ShooterConstants.hoodGearRatio;
    hoodMotor.getConfigurator().apply(hoodConfig);

    var indexerConfig = new TalonFXConfiguration();
    indexerConfig.MotorOutput.Inverted =
        ShooterConstants.indexerInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // brake to prevent rollback
    indexerConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.indexerCurrentLimit;
    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerConfig.Slot0.kP = ShooterConstants.indexerKp;
    indexerConfig.Slot0.kI = ShooterConstants.indexerKi;
    indexerConfig.Slot0.kD = ShooterConstants.indexerKd;
    indexerConfig.Slot0.kV = ShooterConstants.indexerKv;
    indexerMotor.getConfigurator().apply(indexerConfig);

    leftFlywheelVelocity = leftFlywheelMotor.getVelocity();
    leftFlywheelAppliedVolts = leftFlywheelMotor.getMotorVoltage();
    leftFlywheelCurrent = leftFlywheelMotor.getSupplyCurrent();
    leftFlywheelTemp = leftFlywheelMotor.getDeviceTemp();

    rightFlywheelVelocity = rightFlywheelMotor.getVelocity();
    rightFlywheelAppliedVolts = rightFlywheelMotor.getMotorVoltage();
    rightFlywheelCurrent = rightFlywheelMotor.getSupplyCurrent();
    rightFlywheelTemp = rightFlywheelMotor.getDeviceTemp();

    hoodPosition = hoodMotor.getPosition();
    hoodVelocity = hoodMotor.getVelocity();
    hoodAppliedVolts = hoodMotor.getMotorVoltage();
    hoodCurrent = hoodMotor.getSupplyCurrent();

    indexerVelocity = indexerMotor.getVelocity();
    indexerAppliedVolts = indexerMotor.getMotorVoltage();
    indexerCurrent = indexerMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, // 50hz update rate
        leftFlywheelVelocity,
        leftFlywheelAppliedVolts,
        leftFlywheelCurrent,
        leftFlywheelTemp,
        rightFlywheelVelocity,
        rightFlywheelAppliedVolts,
        rightFlywheelCurrent,
        rightFlywheelTemp,
        hoodPosition,
        hoodVelocity,
        hoodAppliedVolts,
        hoodCurrent,
        indexerVelocity,
        indexerAppliedVolts,
        indexerCurrent);

    leftFlywheelMotor.optimizeBusUtilization();
    rightFlywheelMotor.optimizeBusUtilization();
    hoodMotor.optimizeBusUtilization();
    indexerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftFlywheelVelocity,
        leftFlywheelAppliedVolts,
        leftFlywheelCurrent,
        leftFlywheelTemp,
        rightFlywheelVelocity,
        rightFlywheelAppliedVolts,
        rightFlywheelCurrent,
        rightFlywheelTemp,
        hoodPosition,
        hoodVelocity,
        hoodAppliedVolts,
        hoodCurrent,
        indexerVelocity,
        indexerAppliedVolts,
        indexerCurrent);

    inputs.leftFlywheelConnected = leftFlywheelMotor.isConnected();
    inputs.leftFlywheelVelocityRPM = leftFlywheelVelocity.getValueAsDouble() * 60.0; // rps to rpm
    inputs.leftFlywheelAppliedVolts = leftFlywheelAppliedVolts.getValueAsDouble();
    inputs.leftFlywheelCurrentAmps = leftFlywheelCurrent.getValueAsDouble();
    inputs.leftFlywheelTempCelsius = leftFlywheelTemp.getValueAsDouble();

    inputs.rightFlywheelConnected = rightFlywheelMotor.isConnected();
    inputs.rightFlywheelVelocityRPM = rightFlywheelVelocity.getValueAsDouble() * 60.0; // rps to rpm
    inputs.rightFlywheelAppliedVolts = rightFlywheelAppliedVolts.getValueAsDouble();
    inputs.rightFlywheelCurrentAmps = rightFlywheelCurrent.getValueAsDouble();
    inputs.rightFlywheelTempCelsius = rightFlywheelTemp.getValueAsDouble();

    inputs.hoodConnected = hoodMotor.isConnected();
    inputs.hoodPositionDeg = hoodPosition.getValueAsDouble() * 360.0; // rotations to degrees
    inputs.hoodVelocityDegPerSec = hoodVelocity.getValueAsDouble() * 360.0; // rps to deg/s
    inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
    inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();

    inputs.indexerConnected = indexerMotor.isConnected();
    inputs.indexerVelocityRPM = indexerVelocity.getValueAsDouble() * 60.0; // rps to rpm
    inputs.indexerAppliedVolts = indexerAppliedVolts.getValueAsDouble();
    inputs.indexerCurrentAmps = indexerCurrent.getValueAsDouble();
  }

  @Override
  public void setLeftFlywheelVelocity(double velocityRPM) {
    double velocityRPS = velocityRPM / 60.0; // rpm to rps
    leftFlywheelMotor.setControl(flywheelVelocityRequest.withVelocity(velocityRPS));
  }

  @Override
  public void setRightFlywheelVelocity(double velocityRPM) {
    double velocityRPS = velocityRPM / 60.0; // rpm to rps
    rightFlywheelMotor.setControl(flywheelVelocityRequest.withVelocity(velocityRPS));
  }

  @Override
  public void setFlywheelVelocity(double velocityRPM) {
    setLeftFlywheelVelocity(velocityRPM);
    setRightFlywheelVelocity(velocityRPM);
  }

  @Override
  public void setLeftFlywheelOpenLoop(double volts) {
    leftFlywheelMotor.setControl(flywheelVoltageRequest.withOutput(volts));
  }

  @Override
  public void setRightFlywheelOpenLoop(double volts) {
    rightFlywheelMotor.setControl(flywheelVoltageRequest.withOutput(volts));
  }

  @Override
  public void setHoodPosition(double angleDeg) {
    double rotations = angleDeg / 360.0; // degrees to rotations
    hoodMotor.setControl(hoodPositionRequest.withPosition(rotations));
  }

  @Override
  public void setHoodOpenLoop(double volts) {
    hoodMotor.setControl(hoodVoltageRequest.withOutput(volts));
  }

  @Override
  public void setIndexerVelocity(double velocityRPM) {
    double velocityRPS = velocityRPM / 60.0; // rpm to rps
    indexerMotor.setControl(indexerVelocityRequest.withVelocity(velocityRPS));
  }

  @Override
  public void setIndexerOpenLoop(double volts) {
    indexerMotor.setControl(indexerVoltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    leftFlywheelMotor.stopMotor();
    rightFlywheelMotor.stopMotor();
    hoodMotor.stopMotor();
    indexerMotor.stopMotor();
  }
}
