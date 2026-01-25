package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command that waits for the shooter to reach target speed.
 * Used to confirm shooter is ready before shooting.
 * 
 * This command finishes once the shooter reaches the target RPM.
 * Useful in sequences like: SpinUp → WaitUntilReady → Feed
 */
public class ShootWhenReady extends Command {
    
    private final ShooterSubsystem shooterSubsystem;
    
    /**
     * Creates a new ShootWhenReady command.
     * 
     * @param shooterSubsystem The shooter subsystem
     */
    public ShootWhenReady(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        
        // Don't require the shooter, just monitor it
        // This lets it run alongside SpinUpShooter and/or AutoAimShooter
    }
    
    @Override
    public void initialize() {
        System.out.println("Waiting for shooter to reach speed...");
    }
    
    @Override
    public void execute() {
        // Just wait: the checking happens in isFinished()
    }
    
    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            System.out.println("Shooter is ready to shoot!");
        } else {
            System.out.println("ShootWhenReady interrupted before reaching speed");
        }
    }
    
    @Override
    public boolean isFinished() {
        // Finish when shooter reaches target speed
        return shooterSubsystem.isAtSpeed();
    }
}