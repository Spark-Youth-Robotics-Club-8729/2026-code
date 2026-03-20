// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Hardware IO implementation: Kraken X60 for roller, SparkMax (NEO) for slapdown. */
public class IntakeIOHardware implements IntakeIO {

  // ---------------------------------------------------------------------------
  // Roller — Kraken X60 via Phoenix 6
  // ---------------------------------------------------------------------------
  private final TalonFX rollerMotor;

  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerCurrent;
  private final StatusSignal<Temperature> rollerTemp;

  private final VoltageOut rollerVoltageControl = new VoltageOut(0);

  private final SparkMax slapdownMotor;
  private final SparkAbsoluteEncoder slapdownAbsEncoder;
  private final SparkClosedLoopController slapdownController;

  // Track last PID gains so we only reconfigure when they change
  private double lastKp = -1.0;
  private double lastKd = -1.0;

  public IntakeIOHardware() {
    // ---- Roller (Kraken X60) ----
    CANBus canBusInstance = canBus.equals("rio") ? CANBus.roboRIO() : new CANBus(canBus);
    rollerMotor = new TalonFX(rollerMotorID, canBusInstance);

    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerMotor.getConfigurator().apply(rollerConfig);

    rollerVelocity = rollerMotor.getVelocity();
    rollerAppliedVolts = rollerMotor.getMotorVoltage();
    rollerCurrent = rollerMotor.getSupplyCurrent();
    rollerTemp = rollerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, rollerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, rollerAppliedVolts, rollerCurrent, rollerTemp);
    rollerMotor.optimizeBusUtilization();

    // ---- Slapdown (SparkMax + NEO + Through Bore absolute encoder) ----
    slapdownMotor = new SparkMax(slapdownMotorID, MotorType.kBrushless);
    slapdownAbsEncoder = slapdownMotor.getAbsoluteEncoder();
    slapdownController = slapdownMotor.getClosedLoopController();

    SparkMaxConfig slapdownConfig = new SparkMaxConfig();
    slapdownConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(false); // flip to true if arm moves the wrong direction
    slapdownConfig
        .absoluteEncoder
        // Convert rotations → radians so all position reads/setpoints use radians directly.
        // The Through Bore is on the output shaft, so no gear ratio needed here.
        .positionConversionFactor(2.0 * Math.PI)
        .velocityConversionFactor((2.0 * Math.PI) / 60.0)
        // Zero offset in rotations: raw encoder value when the arm is at the UP position.
        .zeroOffset(slapdownEncoderOffset)
        .inverted(false);
    slapdownConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(slapdownUpKp)
        .d(slapdownUpKd)
        .outputRange(-0.3, 0.3);
    slapdownConfig
        .softLimit
        .forwardSoftLimit((float) slapdownDownAngleRad)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit((float) slapdownUpAngleRad)
        .reverseSoftLimitEnabled(true);

    slapdownMotor.configure(
        slapdownConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    lastKp = slapdownUpKp;
    lastKd = slapdownUpKd;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Roller
    inputs.rollerConnected =
        BaseStatusSignal.refreshAll(rollerVelocity, rollerAppliedVolts, rollerCurrent, rollerTemp)
            .isOK();
    inputs.rollerVelocityRPM = rollerVelocity.getValueAsDouble() * 60.0;
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
    inputs.rollerTempCelsius = rollerTemp.getValueAsDouble();

    // Slapdown
    inputs.slapdownConnected = !slapdownMotor.hasActiveFault();
    inputs.slapdownPositionRad = slapdownAbsEncoder.getPosition(); // radians via conversion factor
    inputs.slapdownVelocityRadPerSec = slapdownAbsEncoder.getVelocity(); // rad/s
    inputs.slapdownAppliedVolts = slapdownMotor.getAppliedOutput() * slapdownMotor.getBusVoltage();
    inputs.slapdownCurrentAmps = slapdownMotor.getOutputCurrent();
    inputs.slapdownTempCelsius = slapdownMotor.getMotorTemperature();
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    // Roller (always open-loop voltage)
    rollerMotor.setControl(rollerVoltageControl.withOutput(outputs.rollerVolts));

    // Reconfigure SparkMax PID gains only when they actually change
    if (outputs.kP != lastKp || outputs.kD != lastKd) {
      SparkMaxConfig update = new SparkMaxConfig();
      update.closedLoop.p(outputs.kP).d(outputs.kD);
      slapdownMotor.configure(
          update, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      lastKp = outputs.kP;
      lastKd = outputs.kD;
    }

    // Slapdown
    switch (outputs.slapdownMode) {
      case BRAKE -> slapdownMotor.stopMotor();

      case COAST -> {
        SparkMaxConfig coastConfig = new SparkMaxConfig();
        coastConfig.idleMode(IdleMode.kCoast);
        slapdownMotor.configure(
            coastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        slapdownMotor.stopMotor();
      }

      case OPEN_LOOP -> slapdownMotor.setVoltage(outputs.slapdownVolts);

      case CLOSED_LOOP ->
          slapdownController.setSetpoint(
              outputs.slapdownPositionRad, // radians, matches absolute encoder conversion factor
              ControlType.kPosition,
              ClosedLoopSlot.kSlot0,
              0.0);
    }
  }
}
