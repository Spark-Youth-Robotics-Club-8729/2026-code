// ManualAuto.java
package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class ManualAuto {

  public static Command simpleAuto(Drive drive) {
    return Commands.sequence(
        // Step 1: Orient wheels straight (forward) and hold for 0.5s to let them snap into place
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0)), drive)
            .withTimeout(0.5),

        // Step 2: Drive backwards for 2 seconds
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1.5, 0.0, 0.0)), drive)
            .withTimeout(2.0),

        // Step 3: Drive forward for 5 seconds
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1.5, 0.0, 0.0)), drive)
            .withTimeout(5.0),

        // Step 4: Stop
        Commands.runOnce(() -> drive.stop(), drive));
  }
}
