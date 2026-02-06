package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Subsystem that controls the shooter wheels.
 * Uses PID velocity control to maintain target RPM.
 */
public class ShooterSubsystem extends SubsystemBase {
    
    // Hardware
    private final SparkMax motor;
    private final SparkClosedLoopController pidController;
    private final RelativeEncoder encoder;
    
    // State
    private double targetRPM = 0.0;
    
    /**
     * Creates a new ShooterSubsystem.
     */
    @SuppressWarnings("removal")  // Suppress deprecation warning for configure() (it was pmo)
    public ShooterSubsystem() {
        // Initialize motor
        motor = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        
        // Create config object
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Configure PID 
        config.closedLoop
            .p(Constants.ShooterConstants.SHOOTER_KP)
            .i(Constants.ShooterConstants.SHOOTER_KI)
            .d(Constants.ShooterConstants.SHOOTER_KD)
            .outputRange(-1, 1);
        
        // Configure velocity feedforward
        config.closedLoop.feedForward
            .kV(Constants.ShooterConstants.SHOOTER_KF);
        
        // Set limits for safety
        config.smartCurrentLimit(40);
        
        // Apply config
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Get PID controller and encoder
        pidController = motor.getClosedLoopController();
        encoder = motor.getEncoder();
    }
    
    /**
     * Set the target RPM for the shooter.
     * 
     * @param rpm The desired RPM
     */
    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        pidController.setSetpoint(rpm, ControlType.kVelocity);
    }
    
    /**
     * Stop the shooter motor.
     */
    public void stop() {
        targetRPM = 0.0;
        motor.set(0);
    }
    
    /**
     * Get the current shooter RPM from the encoder.
     * 
     * @return Current RPM
     */
    public double getCurrentRPM() {
        return encoder.getVelocity();
    }
    
    /**
     * Check if the shooter is at the target speed.
     * 
     * @return True if within tolerance
     */
    public boolean isAtSpeed() {
        return Math.abs(getCurrentRPM() - targetRPM) < Constants.ShooterConstants.SHOOTER_TOLERANCE_RPM;
    }
    
    /**
     * Get the target RPM.
     * 
     * @return Target RPM
     */
    public double getTargetRPM() {
        return targetRPM;
    }
    
    /**
     * Get the RPM error (target - current).
     * 
     * @return RPM error
     */
    public double getRPMError() {
        return targetRPM - getCurrentRPM();
    }
    
    @Override
    public void periodic() {
        // This method is called every 20ms
        // Update dashboard with shooter status
        SmartDashboard.putNumber("Shooter/Current RPM", getCurrentRPM());
        SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter/RPM Error", getRPMError());
        SmartDashboard.putBoolean("Shooter/At Speed", isAtSpeed());
        SmartDashboard.putNumber("Shooter/Motor Current", motor.getOutputCurrent());
    }
}