package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX rollerMotor;
  private final SparkMax slapdownMotor;
  private final RelativeEncoder slapdownEncoder;

  private final PIDController slapdownPID =
      new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

  private boolean manualMode = false;

  @SuppressWarnings("removal")
  public IntakeSubsystem() {

    // Roller motor (CTRE)
    rollerMotor = new TalonFX(IntakeConstants.kRollerMotorID);

    // Slapdown motor (REV SparkMax)
    slapdownMotor = new SparkMax(IntakeConstants.kSlapdownMotorID, MotorType.kBrushless);
    slapdownEncoder = slapdownMotor.getEncoder();

    // -------- CTRE TalonFX Config --------
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerMotor.getConfigurator().apply(configs);

    // -------- REV SparkMax Config --------
    SparkMaxConfig slapConfig = new SparkMaxConfig();
    slapConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    slapdownMotor.configure(
        slapConfig,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters
    );

    // PID settings
    slapdownPID.setTolerance(1.0);

    // Set encoder starting position
    slapdownEncoder.setPosition(IntakeConstants.kUpPosition);
  }

  // ---------------------------
  // Roller Controls
  // ---------------------------

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public void stopRollers() {
    setRollerSpeed(0);
  }

  // ---------------------------
  // Slapdown Manual Control
  // ---------------------------

  public void setSlapdownManual(double speed) {
    manualMode = true;

    double position = getSlapdownPosition();

    // Prevent driving past limits
    if ((position <= IntakeConstants.kMinPosition && speed < 0) ||
        (position >= IntakeConstants.kMaxPosition && speed > 0)) {
      slapdownMotor.set(0);
      return;
    }

    speed = MathUtil.clamp(speed, -IntakeConstants.kManualMaxOutput, IntakeConstants.kManualMaxOutput);
    slapdownMotor.set(speed);
  }

  public void stopSlapdown() {
    slapdownMotor.set(0);
  }

  // ---------------------------
  // PID Position Control
  // ---------------------------

  public void goToPosition(double targetPosition) {
    manualMode = false;

    targetPosition = MathUtil.clamp(
        targetPosition,
        IntakeConstants.kMinPosition,
        IntakeConstants.kMaxPosition
    );

    double output = slapdownPID.calculate(getSlapdownPosition(), targetPosition);
    output = MathUtil.clamp(output, -IntakeConstants.kMaxPIDOutput, IntakeConstants.kMaxPIDOutput);

    slapdownMotor.set(output);
  }

  public boolean atTarget() {
    return slapdownPID.atSetpoint();
  }

  public void resetPID() {
    slapdownPID.reset();
  }

  // ---------------------------
  // Encoder Utilities
  // ---------------------------

  public double getSlapdownPosition() {
    return slapdownEncoder.getPosition();
  }

  public void resetEncoderTo(double position) {
    slapdownEncoder.setPosition(position);
  }

  public void resetEncoderToUp() {
    slapdownEncoder.setPosition(IntakeConstants.kUpPosition);
  }

  // ---------------------------
  // Telemetry
  // ---------------------------

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Position", getSlapdownPosition());
    SmartDashboard.putBoolean("Intake At Target", atTarget());
    SmartDashboard.putBoolean("Intake Manual Mode", manualMode);
    SmartDashboard.putNumber("Intake PID Error", slapdownPID.getPositionError());
  }
}
