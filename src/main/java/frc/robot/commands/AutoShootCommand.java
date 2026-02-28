package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.Index.IndexerSubsystem;


public class AutoShootCommand extends Command {

  private final Drive drive;
  private final Shooter shooter;
  private final IndexerSubsystem indexer;

  // HUB FIELD POSITION (meters)
  // TODO: Replace with official 2026 coordinates later
  private final Translation2d hubPosition =
      new Translation2d(8.25, 4.10);

  public AutoShootCommand(
      Drive drive,
      Shooter shooter,
      IndexerSubsystem indexer) {

    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;

    addRequirements(shooter, indexer, drive);
  }

  @Override
  public void execute() {

    
    // Calculate shooting parameters based on robot position
    ShotCalculator calculator = ShotCalculator.getInstance();
    var params = calculator.calculate();

    // If robot is outside shooting range → stop shooter
    if (!params.isValid()) {
      shooter.stop();
      indexer.stoprotate();
      return;
    }


    //Automatically set hood angle + flywheel speed
    shooter.applyShootingParameters(
        params.hoodAngleRad(),
        params.flywheelSpeedRPM());


    // rotate robot toward HUB

    var pose = drive.getPose();

    Translation2d robotToHub =
        hubPosition.minus(pose.getTranslation());

    double desiredAngle =
        Math.atan2(robotToHub.getY(), robotToHub.getX());

    drive.autoRotateToAngle(desiredAngle);

    
    // Feed note ONLY when shooter is ready
    
    if (shooter.isReadyToShoot()) {

      // Green feeder wheels inside shooter
      shooter.feedNote();

      // Main indexer pushing balls upward
      indexer.rotate();

    } else {

      shooter.stopFeeder();
      indexer.stoprotate();
    }
  }

  @Override
  public void end(boolean interrupted) {

    // Stop everything when button released
    shooter.stop();
    indexer.stoprotate();
  }

  @Override
  public boolean isFinished() {
    return false; // runs while button held
  }
}