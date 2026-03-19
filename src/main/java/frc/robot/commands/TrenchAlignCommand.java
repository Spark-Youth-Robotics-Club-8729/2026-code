package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * Horizontally aligns the robot to be centered on the nearest trench AprilTag using Limelight TX
 * for visual servoing. The robot strafes left/right without rotating.
 *
 * <p>Aiming strategy:
 *
 * <ul>
 *   <li>If a trench tag is visible: proportional TX control strafes the robot toward center.
 *   <li>If no target is visible: robot holds position.
 * </ul>
 *
 * <p>Bind via {@code .whileTrue()}.
 */
public class TrenchAlignCommand extends Command {
  // Proportional gain for horizontal strafe — tune if robot oscillates (lower) or undershoots
  // (higher)
  private static final double KP_STRAFE = 0.035;
  // Minimum strafe command to overcome static friction (m/s)
  private static final double MIN_STRAFE_COMMAND_M_S = 0.1;
  // Dead-zone: don't correct if within ±2 degrees
  private static final double AIM_DEAD_BAND_DEG = 2.0;

  private final Drive drive;
  private final Vision vision;
  private final int cameraIndex;

  /**
   * @param drive Drive subsystem.
   * @param vision Vision subsystem (for TX-based alignment).
   * @param cameraIndex Which camera to use for alignment (0 = camera0Name).
   */
  public TrenchAlignCommand(Drive drive, Vision vision, int cameraIndex) {
    this.drive = drive;
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    boolean hasTarget = vision.hasTrenchTarget(cameraIndex);

    double strafeVx = 0.0;

    if (hasTarget) {
      double txDeg = vision.getTargetX(cameraIndex).getDegrees();

      if (Math.abs(txDeg) > AIM_DEAD_BAND_DEG) {
        // TX positive = target is right of crosshair → strafe right (positive vy in robot frame)
        strafeVx = txDeg * KP_STRAFE * drive.getMaxLinearSpeedMetersPerSec();
        strafeVx += Math.copySign(MIN_STRAFE_COMMAND_M_S, strafeVx);
      }
    }

    // Strafe only — no rotation, no forward/back movement
    drive.runVelocity(new ChassisSpeeds(0.0, strafeVx, 0.0));

    boolean aligned =
        hasTarget && Math.abs(vision.getTargetX(cameraIndex).getDegrees()) < AIM_DEAD_BAND_DEG;

    Logger.recordOutput("TrenchAlign/HasTarget", hasTarget);
    Logger.recordOutput(
        "TrenchAlign/TXDeg", hasTarget ? vision.getTargetX(cameraIndex).getDegrees() : 0.0);
    Logger.recordOutput("TrenchAlign/StrafeVyMps", strafeVx);
    Logger.recordOutput("TrenchAlign/Aligned", aligned);
    Logger.recordOutput(
        "TrenchAlign/State", !hasTarget ? "NoTarget" : aligned ? "Aligned" : "Aligning");
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // runs while button held
  }
}
