package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * Automatically aims the robot at the shoot target using Limelight TX for visual servoing, spins up
 * the shooter to the correct speed/hood angle via ShotCalculator, and feeds when ready.
 *
 * <p>Aiming strategy:
 *
 * <ul>
 *   <li>If a Limelight target is visible: proportional TX control rotates the robot toward the
 *       target (fast, direct visual feedback — no field-pose math needed).
 *   <li>If no target is visible: robot seeks by spinning slowly.
 *   <li>If gyro rate is too high (&gt;720 deg/s): feeding is suppressed to avoid shooting while
 *       spinning wildly.
 * </ul>
 *
 * <p>Bind via {@code .whileTrue()}.
 */
public class AutoShootCommand extends Command {
  // ---- Proportional aim gain — tune if robot oscillates (lower) or undershoots (higher) ----
  private static final double KP_AIM = 0.035;
  // Minimum angular command to overcome static friction (rad/s)
  private static final double MIN_AIM_COMMAND_RAD_S = 0.3;
  // Dead-zone: don't correct if within ±1 degree
  private static final double AIM_DEAD_BAND_DEG = 1.0;
  // Seek speed when no target is visible (rad/s)
  private static final double SEEK_SPEED_RAD_S = 1.2;
  // Gyro rate threshold above which we suppress feeding (deg/s)
  private static final double MAX_FEED_GYRO_RATE_DEG_S = 720.0;

  private final Drive drive;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;
  private final int cameraIndex;

  /**
   * @param drive Drive subsystem.
   * @param shooter Shooter subsystem.
   * @param indexer Indexer subsystem.
   * @param vision Vision subsystem (for TX-based aiming).
   * @param cameraIndex Which camera to use for aiming (0 = camera0Name).
   */
  public AutoShootCommand(
      Drive drive, Shooter shooter, Indexer indexer, Vision vision, int cameraIndex) {
    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    addRequirements(drive, shooter, indexer);
  }

  @Override
  public void initialize() {
    // Nothing to reset — proportional controller is stateless
  }

  @Override
  public void execute() {
    // ---- Choose best distance source ----
    // Prefer direct vision distance from hub tags (most accurate);
    // fall back to pose-based calculation if hub tags not visible.
    ShotCalculator.ShootingParameters params;
    boolean usingVisionDistance = vision.hasHubTarget(cameraIndex);

    if (usingVisionDistance) {
      double visionDist = vision.getDistanceToHub(cameraIndex);
      // Drive angle toward the alliance hub from current pose
      var hubPos = ShotCalculator.getInstance().getAllianceHubPosition();
      var robotPos = drive.getPose().getTranslation();
      var driveAngle = hubPos.minus(robotPos).getAngle();
      params = ShotCalculator.getInstance().calculateFromDistance(visionDist, driveAngle);
    } else {
      params = ShotCalculator.getInstance().calculate();
    }

    Logger.recordOutput("AutoShoot/UsingVisionDistance", usingVisionDistance);

    // ---- Spin up hood + flywheels regardless of aiming state ----
    if (params.isValid()) {
      shooter.applyShootingParameters(params.hoodAngleRad(), params.flywheelSpeedRPM());
    } else {
      shooter.stop();
      indexer.setGoal(IndexerGoal.STOP);
      drive.stop();
      Logger.recordOutput("AutoShoot/State", "OutOfRange");
      return;
    }

    // ---- Limelight TX-based visual servoing ----
    double omega;
    boolean hasTarget = vision.hasTarget(cameraIndex);

    if (hasTarget) {
      double txDeg = vision.getTargetX(cameraIndex).getDegrees();
      if (Math.abs(txDeg) < AIM_DEAD_BAND_DEG) {
        omega = 0.0;
      } else {
        // TX positive = target is right of crosshair → rotate CCW (positive omega in WPILib)
        omega = -txDeg * KP_AIM * drive.getMaxAngularSpeedRadPerSec();
        omega += Math.copySign(MIN_AIM_COMMAND_RAD_S, omega);
      }
    } else {
      // No target visible — seek by spinning slowly
      omega = SEEK_SPEED_RAD_S;
    }

    // Robot stays in place, only rotation
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));

    // ---- Gyro rate guard (from Limelight localization example) ----
    double gyroRateDegS = Math.toDegrees(Math.abs(drive.getChassisSpeeds().omegaRadiansPerSecond));
    boolean spinningTooFast = gyroRateDegS > MAX_FEED_GYRO_RATE_DEG_S;

    // ---- Feed only when fully aligned and ready ----
    boolean aimed =
        hasTarget && Math.abs(vision.getTargetX(cameraIndex).getDegrees()) < AIM_DEAD_BAND_DEG;
    boolean readyToFeed = aimed && shooter.isReadyToShoot() && !spinningTooFast;

    if (readyToFeed) {
      shooter.feedNote();
      indexer.setGoal(IndexerGoal.FEED);
      Logger.recordOutput("AutoShoot/State", "Firing");
    } else {
      shooter.stopFeeder();
      indexer.setGoal(IndexerGoal.STOP);
      Logger.recordOutput(
          "AutoShoot/State",
          !hasTarget
              ? "Seeking"
              : spinningTooFast
                  ? "SpinningTooFast"
                  : !shooter.isReadyToShoot() ? "SpinningUp" : "Aligning");
    }

    Logger.recordOutput("AutoShoot/HasTarget", hasTarget);
    Logger.recordOutput(
        "AutoShoot/TXDeg", hasTarget ? vision.getTargetX(cameraIndex).getDegrees() : 0.0);
    Logger.recordOutput("AutoShoot/OmegaRadS", omega);
    Logger.recordOutput("AutoShoot/GyroRateDegS", gyroRateDegS);
    Logger.recordOutput("AutoShoot/Aimed", aimed);
    Logger.recordOutput("AutoShoot/ReadyToFeed", readyToFeed);
    Logger.recordOutput("AutoShoot/DistanceM", params.distanceToTarget());
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    shooter.stopFeeder();
    indexer.setGoal(IndexerGoal.STOP);
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // runs while button held
  }
}
