// TODO: Complete vision subsystem, then uncomment the other lines of code in here. 
// Variable names are placeholders for the moment. They may require modification.


package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.ShooterCalculations;

/**
 * Command that uses vision to automatically aim and spin up the shooter.
 * Continuously calculates required RPM based on distance to target.
 */
public class AutoAimShooter extends Command {
    
    private final ShooterSubsystem shooterSubsystem;
    // private final VisionSubsystem visionSubsystem;
    
    /**
     * Creates a new AutoAimShooter command.
     * 
     * @param shooterSubsystem The shooter subsystem
     * @param visionSubsystem The vision subsystem
     */
    public AutoAimShooter(ShooterSubsystem shooterSubsystem) { // Add VisionSubsystem visionSubsystem once its done
        this.shooterSubsystem = shooterSubsystem;
        // this.visionSubsystem = visionSubsystem;
        
        // This command requires the shooter
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {
        // Turn on Limelight LEDs when command starts
        // visionSubsystem.setLEDOn();
    }
    
    @Override
    public void execute() {
        // Check if we have a valid target
        // if (!visionSubsystem.hasTarget()) {
        //     // No target visible - stop shooter
        //     shooterSubsystem.stop();
        //     return;
        // }
        
        // Get distance to target from vision
        // double distance = visionSubsystem.getDistanceToTarget();
        
        // if (distance < 0) {
        //     // Invalid distance - stop shooter
        //     shooterSubsystem.stop();
        //     return;
        // }
        
        // Calculate required RPM using ShooterCalculations (the physics)
        // double requiredRPM = ShooterCalculations.calculateRPM(distance);
        
        // Set shooter to calculated RPM
        // shooterSubsystem.setTargetRPM(requiredRPM);
    }
    
    // @Override
    // public void end(boolean interrupted) {
    //     // Turn off LEDs and stop shooter when command ends
    //     visionSubsystem.setLEDOff();
    //     shooterSubsystem.stop();
    // }
    
    @Override
    public boolean isFinished() {
        // This command runs until interrupted
        return false;
    }
}