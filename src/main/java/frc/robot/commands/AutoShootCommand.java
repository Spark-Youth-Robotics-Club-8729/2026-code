package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator;


/**
 * Automatically aims the robot at the shoot target, spins up the shooter to the
 * correct speed/hood angle, and feeds when ready. Runs while button held.
 *
 * <p>Uses the same ProfiledPIDController angle-control approach as
 * {@link DriveCommands#joystickDriveAtAngle}, but with zero translation (robot
 * stays still). Bind via {@code .whileTrue()}.
 */
public class AutoShootCommand extends Command {

  // Match the PID constants used in DriveCommands
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;     // rad/s
  private static final double ANGLE_MAX_ACCELERATION = 20.0; // rad/s²

  private final Drive drive;
  private final Shooter shooter;
  private final Indexer indexer;

  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          ANGLE_KP,
          0.0,
          ANGLE_KD,
          new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

  public AutoShootCommand(Drive drive, Shooter shooter, Indexer indexer) {
    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;
    addRequirements(drive, shooter, indexer);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    angleController.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    var params = ShotCalculator.getInstance().calculate();

    // Outside valid range — stop everything and wait
    if (!params.isValid()) {
      shooter.stop();
      indexer.setGoal(IndexerGoal.STOP);
      drive.stop();
      return;
    }

    // Set hood + flywheels via the convenience method
    shooter.applyShootingParameters(params.hoodAngleRad(), params.flywheelSpeedRPM());

    // Rotate toward target using ProfiledPIDController — no translation
    double omega =
        angleController.calculate(
            drive.getRotation().getRadians(), params.driveAngle().getRadians());
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));

    // Feed only when fully ready (flywheels at speed + hood at angle)
    if (shooter.isReadyToShoot()) {
      shooter.feedNote();
      indexer.setGoal(IndexerGoal.FEED);
    } else {
      shooter.stopFeeder();
      indexer.setGoal(IndexerGoal.STOP);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.setGoal(IndexerGoal.STOP);
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // runs while button held
  }
}