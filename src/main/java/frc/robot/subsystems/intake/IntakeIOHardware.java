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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
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

  // ---------------------------------------------------------------------------
  // Slapdown — SparkMax (NEO brushless) via REVLib
  // ---------------------------------------------------------------------------
  private final SparkMax slapdownMotor;
  private final RelativeEncoder slapdownEncoder;
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

    // ---- Slapdown (SparkMax + NEO) ----
    slapdownMotor = new SparkMax(slapdownMotorID, MotorType.kBrushless);
    slapdownEncoder = slapdownMotor.getEncoder();
    slapdownController = slapdownMotor.getClosedLoopController();

    SparkMaxConfig slapdownConfig = new SparkMaxConfig();
    slapdownConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(false); // TODO: flip to true if arm moves the wrong direction
    slapdownConfig
        .encoder
        // Conversion factors so all encoder reads return radians / rad/s directly
        .positionConversionFactor((2.0 * Math.PI) / slapdownGearRatio)
        .velocityConversionFactor((2.0 * Math.PI) / (slapdownGearRatio * 60.0));
    slapdownConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(slapdownKp)
        .d(slapdownKd)
        .outputRange(-1.0, 1.0);
    slapdownConfig
        .softLimit
        .forwardSoftLimit((float) slapdownDownAngleRad)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit((float) slapdownUpAngleRad)
        .reverseSoftLimitEnabled(true);

    // Use top-level com.revrobotics.ResetMode / PersistMode (not SparkBase.* — those are
    // deprecated)
    slapdownMotor.configure(
        slapdownConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Seed encoder at the retracted (up) position on startup
    slapdownEncoder.setPosition(slapdownUpAngleRad);

    lastKp = slapdownKp;
    lastKd = slapdownKd;
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

    // Slapdown — hasActiveFault() is the current non-deprecated connection check
    inputs.slapdownConnected = !slapdownMotor.hasActiveFault();
    inputs.slapdownPositionRad = slapdownEncoder.getPosition(); // radians (via conversion factor)
    inputs.slapdownVelocityRadPerSec =
        slapdownEncoder.getVelocity(); // rad/s (via conversion factor)
    inputs.slapdownAppliedVolts = slapdownMotor.getAppliedOutput() * slapdownMotor.getBusVoltage();
    inputs.slapdownCurrentAmps = slapdownMotor.getOutputCurrent();
    inputs.slapdownTempCelsius = slapdownMotor.getMotorTemperature();
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    // Roller (always open-loop voltage)
    rollerMotor.setControl(rollerVoltageControl.withOutput(outputs.rollerVolts));

    // Reconfigure SparkMax PID gains only when they actually change —
    // calling configure() every loop is slow and unnecessary
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
      // setSetpoint() is the non-deprecated replacement for setReference()
      slapdownController.setSetpoint(
          outputs.slapdownPositionRad, // already in radians via conversion factor
          ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          0.0); // no arbitrary feedforward
    }
  }
}
