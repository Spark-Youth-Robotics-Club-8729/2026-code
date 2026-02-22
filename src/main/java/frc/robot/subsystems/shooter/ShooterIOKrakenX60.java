// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** IO implementation for Kraken X60 motors using Phoenix 6. */
public class ShooterIOKrakenX60 implements ShooterIO {
  private final TalonFX leftFlywheelMotor;
  private final TalonFX rightFlywheelMotor;
  private final TalonFX hoodMotor;
  private final TalonFX indexerMotor;

  // Status signals - left flywheel
  private final StatusSignal<AngularVelocity> leftFlywheelVelocity;
  private final StatusSignal<Voltage> leftFlywheelAppliedVolts;
  private final StatusSignal<Current> leftFlywheelCurrent;
  private final StatusSignal<Temperature> leftFlywheelTemp;

  // Status signals - right flywheel
  private final StatusSignal<AngularVelocity> rightFlywheelVelocity;
  private final StatusSignal<Voltage> rightFlywheelAppliedVolts;
  private final StatusSignal<Current> rightFlywheelCurrent;
  private final StatusSignal<Temperature> rightFlywheelTemp;

  // Status signals - hood (uses built-in TalonFX encoder)
  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<AngularVelocity> hoodVelocity;
  private final StatusSignal<Voltage> hoodAppliedVolts;
  private final StatusSignal<Current> hoodCurrent;
  private final StatusSignal<Temperature> hoodTemp;

  // Status signals - indexer
  private final StatusSignal<AngularVelocity> indexerVelocity;
  private final StatusSignal<Voltage> indexerAppliedVolts;
  private final StatusSignal<Current> indexerCurrent;
  private final StatusSignal<Temperature> indexerTemp;

  // Control requests
  private final VelocityVoltage leftFlywheelVelocityControl = new VelocityVoltage(0);
  private final VelocityVoltage rightFlywheelVelocityControl = new VelocityVoltage(0);
  private final PositionVoltage hoodPositionControl = new PositionVoltage(0);
  private final VelocityVoltage indexerVelocityControl = new VelocityVoltage(0);
  private final VoltageOut voltageControl = new VoltageOut(0);

  public ShooterIOKrakenX60() {
    CANBus canBusInstance = canBus.equals("rio") ? CANBus.roboRIO() : new CANBus(canBus);

    leftFlywheelMotor = new TalonFX(leftFlywheelID, canBusInstance);
    rightFlywheelMotor = new TalonFX(rightFlywheelID, canBusInstance);
    hoodMotor = new TalonFX(hoodMotorID, canBusInstance);
    indexerMotor = new TalonFX(indexerMotorID, canBusInstance);

    // -------------------------------------------------------------------------
    // Left flywheel — spins counter-clockwise to shoot
    // -------------------------------------------------------------------------
    var leftConfig = new TalonFXConfiguration();
    leftConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftConfig.Feedback.SensorToMechanismRatio = flywheelGearRatio; // 1:1
    leftConfig.Slot0.kP = flywheelKp;
    leftConfig.Slot0.kI = flywheelKi;
    leftConfig.Slot0.kD = flywheelKd;
    leftConfig.Slot0.kV = flywheelKv;
    leftFlywheelMotor.getConfigurator().apply(leftConfig);

    // -------------------------------------------------------------------------
    // Right flywheel — spins clockwise to shoot (opposite of left)
    // -------------------------------------------------------------------------
    var rightConfig = new TalonFXConfiguration();
    rightConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightConfig.Feedback.SensorToMechanismRatio = flywheelGearRatio; // 1:1
    rightConfig.Slot0.kP = flywheelKp;
    rightConfig.Slot0.kI = flywheelKi;
    rightConfig.Slot0.kD = flywheelKd;
    rightConfig.Slot0.kV = flywheelKv;
    rightFlywheelMotor.getConfigurator().apply(rightConfig);

    // -------------------------------------------------------------------------
    // Hood motor — built-in TalonFX encoder, 13.7778:1 reduction
    // SensorToMechanismRatio tells Phoenix that 13.7778 motor rotations = 1
    // mechanism rotation, so all position/velocity reads are in mechanism units.
    // -------------------------------------------------------------------------
    var hoodConfig = new TalonFXConfiguration();
    hoodConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.Feedback.SensorToMechanismRatio = hoodGearRatio; // 13.7778
    hoodConfig.Slot0.kP = hoodKp;
    hoodConfig.Slot0.kI = hoodKi;
    hoodConfig.Slot0.kD = hoodKd;
    // Software limits in mechanism rotations (convert radians → rotations)
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = hoodMaxAngleRad / (2.0 * Math.PI);
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = hoodMinAngleRad / (2.0 * Math.PI);
    hoodMotor.getConfigurator().apply(hoodConfig);

    // -------------------------------------------------------------------------
    // Indexer motor — counter-clockwise = positive = feeds ball up to shooter
    // Gear ratio 1:1.375 stored as motor/mechanism = 1/1.375 ≈ 0.7273
    // -------------------------------------------------------------------------
    var indexerConfig = new TalonFXConfiguration();
    indexerConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    // CCW_Positive means positive setpoint spins counter-clockwise → feeds ball up
    indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexerConfig.Feedback.SensorToMechanismRatio = indexerGearRatio; // ≈ 0.7273
    indexerConfig.Slot0.kP = indexerKp;
    indexerConfig.Slot0.kI = indexerKi;
    indexerConfig.Slot0.kD = indexerKd;
    indexerConfig.Slot0.kV = indexerKv;
    indexerMotor.getConfigurator().apply(indexerConfig);

    // -------------------------------------------------------------------------
    // Status signals
    // -------------------------------------------------------------------------
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
    hoodTemp = hoodMotor.getDeviceTemp();

    indexerVelocity = indexerMotor.getVelocity();
    indexerAppliedVolts = indexerMotor.getMotorVoltage();
    indexerCurrent = indexerMotor.getSupplyCurrent();
    indexerTemp = indexerMotor.getDeviceTemp();

    // High-frequency signals (50 Hz)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftFlywheelVelocity,
        rightFlywheelVelocity,
        hoodPosition,
        hoodVelocity,
        indexerVelocity);

    // Low-frequency telemetry signals (4 Hz)
    BaseStatusSignal.setUpdateFrequencyForAll(
        4.0,
        leftFlywheelAppliedVolts,
        leftFlywheelCurrent,
        leftFlywheelTemp,
        rightFlywheelAppliedVolts,
        rightFlywheelCurrent,
        rightFlywheelTemp,
        hoodAppliedVolts,
        hoodCurrent,
        hoodTemp,
        indexerAppliedVolts,
        indexerCurrent,
        indexerTemp);

    leftFlywheelMotor.optimizeBusUtilization();
    rightFlywheelMotor.optimizeBusUtilization();
    hoodMotor.optimizeBusUtilization();
    indexerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Refresh all signals and use isOK() to detect disconnections
    inputs.leftFlywheelConnected =
        BaseStatusSignal.refreshAll(
                leftFlywheelVelocity,
                leftFlywheelAppliedVolts,
                leftFlywheelCurrent,
                leftFlywheelTemp)
            .isOK();

    inputs.rightFlywheelConnected =
        BaseStatusSignal.refreshAll(
                rightFlywheelVelocity,
                rightFlywheelAppliedVolts,
                rightFlywheelCurrent,
                rightFlywheelTemp)
            .isOK();

    inputs.hoodConnected =
        BaseStatusSignal.refreshAll(
                hoodPosition, hoodVelocity, hoodAppliedVolts, hoodCurrent, hoodTemp)
            .isOK();

    inputs.indexerConnected =
        BaseStatusSignal.refreshAll(
                indexerVelocity, indexerAppliedVolts, indexerCurrent, indexerTemp)
            .isOK();

    // Left flywheel (rotations/sec → RPM)
    inputs.leftFlywheelVelocityRPM = leftFlywheelVelocity.getValueAsDouble() * 60.0;
    inputs.leftFlywheelAppliedVolts = leftFlywheelAppliedVolts.getValueAsDouble();
    inputs.leftFlywheelCurrentAmps = leftFlywheelCurrent.getValueAsDouble();
    inputs.leftFlywheelTempCelsius = leftFlywheelTemp.getValueAsDouble();

    // Right flywheel (rotations/sec → RPM)
    inputs.rightFlywheelVelocityRPM = rightFlywheelVelocity.getValueAsDouble() * 60.0;
    inputs.rightFlywheelAppliedVolts = rightFlywheelAppliedVolts.getValueAsDouble();
    inputs.rightFlywheelCurrentAmps = rightFlywheelCurrent.getValueAsDouble();
    inputs.rightFlywheelTempCelsius = rightFlywheelTemp.getValueAsDouble();

    // Hood — Phoenix reports in mechanism rotations (after SensorToMechanismRatio),
    // so multiply by 2π to get radians.
    inputs.hoodPositionRad = hoodPosition.getValueAsDouble() * 2.0 * Math.PI;
    inputs.hoodVelocityRadPerSec = hoodVelocity.getValueAsDouble() * 2.0 * Math.PI;
    inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
    inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();
    inputs.hoodTempCelsius = hoodTemp.getValueAsDouble();

    // Indexer (rotations/sec → RPM)
    inputs.indexerVelocityRPM = indexerVelocity.getValueAsDouble() * 60.0;
    inputs.indexerAppliedVolts = indexerAppliedVolts.getValueAsDouble();
    inputs.indexerCurrentAmps = indexerCurrent.getValueAsDouble();
    inputs.indexerTempCelsius = indexerTemp.getValueAsDouble();
  }

  @Override
  public void setLeftFlywheelVelocity(double velocityRPM) {
    // Phoenix velocity control is in rotations/sec
    leftFlywheelMotor.setControl(leftFlywheelVelocityControl.withVelocity(velocityRPM / 60.0));
  }

  @Override
  public void setRightFlywheelVelocity(double velocityRPM) {
    rightFlywheelMotor.setControl(rightFlywheelVelocityControl.withVelocity(velocityRPM / 60.0));
  }

  @Override
  public void setHoodPosition(double positionRad) {
    // Phoenix position control is in mechanism rotations (after SensorToMechanismRatio)
    hoodMotor.setControl(hoodPositionControl.withPosition(positionRad / (2.0 * Math.PI)));
  }

  @Override
  public void setIndexerVelocity(double velocityRPM) {
    // Positive RPM → CCW → feeds ball up to shooter
    indexerMotor.setControl(indexerVelocityControl.withVelocity(velocityRPM / 60.0));
  }

  @Override
  public void stop() {
    leftFlywheelMotor.setControl(voltageControl.withOutput(0));
    rightFlywheelMotor.setControl(voltageControl.withOutput(0));
    hoodMotor.setControl(voltageControl.withOutput(0));
    indexerMotor.setControl(voltageControl.withOutput(0));
  }
}
