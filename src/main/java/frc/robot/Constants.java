package frc.robot;

public final class Constants {

  public static final class IntakeConstants {
    // CAN IDs
    public static final int kRollerMotorID = 10;     // TalonFX
    public static final int kSlapdownMotorID = 11;   // SparkMax

    // Roller speeds
    public static final double kIntakeInSpeed = 0.65;
    public static final double kIntakeOutSpeed = -0.65;

    // Slapdown manual speeds
    public static final double kSlapdownDownSpeed = 0.45;
    public static final double kSlapdownUpSpeed = -0.45;

    // Encoder setpoints
    public static final double kUpPosition = 0.0;
    public static final double kDownPosition = 25.0;

    // PID 
    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Safety clamp for PID output
    public static final double kMaxPIDOutput = 0.5;

    // Soft limits (encoder rotations)
    public static final double kMinPosition = 0.0;
    public static final double kMaxPosition = 28.0;

    // Current limits
    public static final int kRollerCurrentLimit = 40;
    public static final int kSlapdownCurrentLimit = 35;

    // Manual safety scaling
    public static final double kManualMaxOutput = 0.6;

    // Telemetry toggle
    public static final boolean kEnableTelemetry = true;
  }
}
