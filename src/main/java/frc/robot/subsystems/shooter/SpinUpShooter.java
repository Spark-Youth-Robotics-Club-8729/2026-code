package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to spin up the shooter to a specific RPM.
 * This command runs continuously until interrupted.
 * 
 * Use this for testing or manual control of the shooter.
 */
public class SpinUpShooter extends Command {
    
    private final ShooterSubsystem shooterSubsystem;
    private final double targetRPM;
    
    /**
     * Creates a new SpinUpShooter command.
     * 
     * @param shooterSubsystem The shooter subsystem to use
     * @param targetRPM The target shooter RPM
     */
    public SpinUpShooter(ShooterSubsystem shooterSubsystem, double targetRPM) {
        this.shooterSubsystem = shooterSubsystem;
        this.targetRPM = targetRPM;
        
        // This command requires the shooter
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {
        // Set the target RPM when command starts
        shooterSubsystem.setTargetRPM(targetRPM);
    }
    
    @Override
    public void execute() {
        // Command maintains the RPM
        // The PID controller in the subsystem handles this automatically
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop shooter when command ends (either interrupted or finished)
        shooterSubsystem.stop();
    }
    
    @Override
    public boolean isFinished() {
        // This command runs until interrupted (returns false = never finishes on its own)
        return false;
    }
}