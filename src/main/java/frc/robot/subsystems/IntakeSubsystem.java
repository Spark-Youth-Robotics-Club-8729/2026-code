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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX rollerMotor;
  private final SparkMax slapdownMotor;
  private final RelativeEncoder slapdownEncoder;

  private final PIDController slapdownPID =
      new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

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

    // PID tolerance (how close is "good enough")
    slapdownPID.setTolerance(1.0);
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public void stopRollers() {
    setRollerSpeed(0);
  }

  public void setSlapdownSpeed(double speed) {
    slapdownMotor.set(speed);
  }

  public void stopSlapdown() {
    setSlapdownSpeed(0);
  }

  public double getSlapdownPosition() {
    return slapdownEncoder.getPosition();
  }

  public void goToPosition(double targetPosition) {
    double output = slapdownPID.calculate(getSlapdownPosition(), targetPosition);
    output = MathUtil.clamp(output, -IntakeConstants.kMaxPIDOutput, IntakeConstants.kMaxPIDOutput);
    slapdownMotor.set(output);
  }

  public boolean atTarget() {
    return slapdownPID.atSetpoint();
  }
}
