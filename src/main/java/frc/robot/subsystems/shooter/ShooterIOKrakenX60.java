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
  private final TalonFX feederMotor; // Renamed from indexerMotor

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

  // Status signals - feeder (green wheels)
  private final StatusSignal<AngularVelocity> feederVelocity;
  private final StatusSignal<Voltage> feederAppliedVolts;
  private final StatusSignal<Current> feederCurrent;
  private final StatusSignal<Temperature> feederTemp;

  // Control requests
  private final VelocityVoltage leftFlywheelVelocityControl = new VelocityVoltage(0);
  private final VelocityVoltage rightFlywheelVelocityControl = new VelocityVoltage(0);
  private final PositionVoltage hoodPositionControl = new PositionVoltage(0);
  private final VelocityVoltage feederVelocityControl = new VelocityVoltage(0);
  private final VoltageOut voltageControl = new VoltageOut(0);

  public ShooterIOKrakenX60() {
    CANBus canBusInstance = canBus.equals("rio") ? CANBus.roboRIO() : new CANBus(canBus);

    leftFlywheelMotor = new TalonFX(leftFlywheelID, canBusInstance);
    rightFlywheelMotor = new TalonFX(rightFlywheelID, canBusInstance);
    hoodMotor = new TalonFX(hoodMotorID, canBusInstance);
    feederMotor = new TalonFX(feederMotorID, canBusInstance);

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
    // Add kS to both flywheel configs to maintain speed when a ball loads
    leftConfig.Slot0.kS = flywheelKs;
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
    rightConfig.Slot0.kS = flywheelKs;
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

    // Seed the hood encoder to the minimum angle (resting/home position) so the
    // controller knows where the hood actually is on startup.
    // Convert radians → rotations (mechanism units after SensorToMechanismRatio).
    hoodMotor.setPosition(hoodMinAngleRad / (2.0 * Math.PI));

    // -------------------------------------------------------------------------
    // Feeder motor (green wheels) — CCW = positive = feeds ball up to shooter
    // -------------------------------------------------------------------------
    var feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederConfig.Feedback.SensorToMechanismRatio = feederGearRatio;
    feederConfig.Slot0.kP = feederKp;
    feederConfig.Slot0.kI = feederKi;
    feederConfig.Slot0.kD = feederKd;
    feederConfig.Slot0.kV = feederKv;
    feederMotor.getConfigurator().apply(feederConfig);

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

    feederVelocity = feederMotor.getVelocity();
    feederAppliedVolts = feederMotor.getMotorVoltage();
    feederCurrent = feederMotor.getSupplyCurrent();
    feederTemp = feederMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftFlywheelVelocity,
        rightFlywheelVelocity,
        hoodPosition,
        hoodVelocity,
        feederVelocity);

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
        feederAppliedVolts,
        feederCurrent,
        feederTemp);

    leftFlywheelMotor.optimizeBusUtilization();
    rightFlywheelMotor.optimizeBusUtilization();
    hoodMotor.optimizeBusUtilization();
    feederMotor.optimizeBusUtilization();
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

    inputs.feederConnected =
        BaseStatusSignal.refreshAll(feederVelocity, feederAppliedVolts, feederCurrent, feederTemp)
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

    // Feeder (rotations/sec → RPM)
    inputs.feederVelocityRPM = feederVelocity.getValueAsDouble() * 60.0;
    inputs.feederAppliedVolts = feederAppliedVolts.getValueAsDouble();
    inputs.feederCurrentAmps = feederCurrent.getValueAsDouble();
    inputs.feederTempCelsius = feederTemp.getValueAsDouble();
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
  public void setFeederVelocity(double velocityRPM) {
    feederMotor.setControl(feederVelocityControl.withVelocity(velocityRPM / 60.0));
  }

  @Override
  public void stop() {
    leftFlywheelMotor.setControl(voltageControl.withOutput(0));
    rightFlywheelMotor.setControl(voltageControl.withOutput(0));
    hoodMotor.setControl(voltageControl.withOutput(0));
    feederMotor.setControl(voltageControl.withOutput(0));
  }
}