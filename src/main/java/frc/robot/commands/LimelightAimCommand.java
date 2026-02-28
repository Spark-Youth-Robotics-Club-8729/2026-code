package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Drives the robot while using proportional control to aim at the best Limelight target. TX from
 * the Limelight replaces the driver's rotation input. If no target is visible, the robot spins
 * slowly to seek.
 *
 * <p>Bind via {@code .whileTrue()}.
 */
public class LimelightAimCommand extends Command {

  // Proportional gain — tune if robot oscillates (lower) or undershoots (higher)
  private static final double KP_AIM = 0.035;
  // Minimum angular command to overcome static friction (rad/s)
  private static final double MIN_COMMAND_RAD_S = 0.3;
  // Dead-zone — don't correct if within ±1 degree
  private static final double AIM_DEAD_BAND_DEG = 1.0;
  // Slow spin rate used when seeking (no target visible), rad/s
  private static final double SEEK_SPEED_RAD_S = 1.2;

  private static final double DEADBAND = 0.1;

  private final Drive drive;
  private final Vision vision;
  private final int cameraIndex;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);

  /**
   * @param drive The drive subsystem.
   * @param vision The vision subsystem.
   * @param cameraIndex Which camera to use (0 = camera0Name).
   * @param xSupplier Driver left-Y (forward/back).
   * @param ySupplier Driver left-X (strafe).
   */
  public LimelightAimCommand(
      Drive drive,
      Vision vision,
      int cameraIndex,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    this.drive = drive;
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    // ---- Translation from driver joystick ----
    double rawX = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
    double rawY = MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND);
    double x = xLimiter.calculate(Math.copySign(rawX * rawX, rawX));
    double y = yLimiter.calculate(Math.copySign(rawY * rawY, rawY));
    double xSpeed = x * drive.getMaxLinearSpeedMetersPerSec();
    double ySpeed = y * drive.getMaxLinearSpeedMetersPerSec();

    // ---- Rotation from Limelight TX ----
    double omega;
    boolean hasTarget = vision.hasTarget(cameraIndex);

    if (hasTarget) {
      double txDeg = vision.getTargetX(cameraIndex).getDegrees();
      if (Math.abs(txDeg) < AIM_DEAD_BAND_DEG) {
        omega = 0.0;
      } else {
        // Proportional — TX positive means target is right, so we rotate CCW (positive omega)
        omega = -txDeg * KP_AIM * drive.getMaxAngularSpeedRadPerSec();
        // Add minimum command to overcome friction
        omega += Math.copySign(MIN_COMMAND_RAD_S, omega);
      }
    } else {
      // No target — seek by spinning slowly
      omega = SEEK_SPEED_RAD_S;
    }

    Logger.recordOutput("LimelightAim/HasTarget", hasTarget);
    Logger.recordOutput("LimelightAim/OmegaRadS", omega);

    // Limelight aiming uses robot-relative (not field-relative) rotation
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            omega,
            isFlipped
                ? drive.getRotation().plus(new edu.wpi.first.math.geometry.Rotation2d(Math.PI))
                : drive.getRotation()));
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
