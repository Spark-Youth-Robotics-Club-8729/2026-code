package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * Uses Limelight TX for aiming and TY for range adjustment simultaneously. The robot drives
 * toward/away from the target and rotates to face it. No driver translation input — robot moves
 * autonomously.
 *
 * <p>Useful for auto-align sequences. Bind via {@code .whileTrue()}.
 */
public class LimelightAimAndRangeCommand extends Command {

  private static final double KP_AIM = 0.035;
  private static final double KP_DISTANCE = 0.1;
  private static final double MIN_AIM_COMMAND_RAD_S = 0.3;
  private static final double AIM_DEAD_BAND_DEG = 1.0;
  private static final double RANGE_DEAD_BAND_DEG = 1.0;

  private final Drive drive;
  private final Vision vision;
  private final int cameraIndex;

  public LimelightAimAndRangeCommand(Drive drive, Vision vision, int cameraIndex) {
    this.drive = drive;
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    if (!vision.hasTarget(cameraIndex)) {
      // Seek — spin slowly until a target appears
      drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 1.0));
      Logger.recordOutput("LimelightAimRange/HasTarget", false);
      return;
    }

    double txDeg = vision.getTargetX(cameraIndex).getDegrees();
    double tyDeg = vision.getTargetY(cameraIndex).getDegrees();

    // Aim
    double omega = 0.0;
    if (Math.abs(txDeg) > AIM_DEAD_BAND_DEG) {
      omega = -txDeg * KP_AIM * drive.getMaxAngularSpeedRadPerSec();
      omega += Math.copySign(MIN_AIM_COMMAND_RAD_S, omega);
    }

    // Range — TY positive = target above crosshair = too close (back up)
    double forwardSpeed = 0.0;
    if (Math.abs(tyDeg) > RANGE_DEAD_BAND_DEG) {
      forwardSpeed = -tyDeg * KP_DISTANCE * drive.getMaxLinearSpeedMetersPerSec();
    }

    Logger.recordOutput("LimelightAimRange/HasTarget", true);
    Logger.recordOutput("LimelightAimRange/TX", txDeg);
    Logger.recordOutput("LimelightAimRange/TY", tyDeg);
    Logger.recordOutput("LimelightAimRange/OmegaRadS", omega);
    Logger.recordOutput("LimelightAimRange/ForwardMps", forwardSpeed);

    // Robot-relative — don't rotate the velocity by field heading
    drive.runVelocity(new ChassisSpeeds(forwardSpeed, 0.0, omega));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
