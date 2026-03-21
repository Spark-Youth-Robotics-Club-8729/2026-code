// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.RollerGoal;
import frc.robot.subsystems.intake.Intake.SlapdownGoal;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

/**
 * System test command (basically just tests all components) Drive: Each swerve module
 * forward/backward/left/right one at a time Intake: Slapdown up/down/jitter, roller intake Indexer:
 * Feed forward Shooter: Multiple flywheel speeds, hood positions, feeder
 */
public class SystemTestCommand extends SequentialCommandGroup {
  private static final double DURATION_SECONDS = 1.0;

  private static final double[] FLYWHEEL_TEST_SPEEDS_RPM = {2000.0, 2500.0, 3100.0};
  private static final double[] HOOD_TEST_ANGLES_DEG = {10.0, 20.0, 35.0};

  public SystemTestCommand(Drive drive, Intake intake, Indexer indexer, Shooter shooter) {
    addCommands(
        Commands.print("=== SYSTEM TEST START ==="),
        Commands.runOnce(
            () -> {
              drive.stop();
              shooter.stop();
              shooter.stopFeeder();
              indexer.setGoal(IndexerGoal.STOP);
            }),
        Commands.waitSeconds(0.5),

        // DRIVE SUBSYSTEM TEST
        Commands.print("--- DRIVE TEST: Swerve Module 0 (Front Left) ---"),
        testSingleModule(drive, 0, "Forward"),
        testSingleModule(drive, 0, "Backward"),
        testSingleModule(drive, 0, "Left"),
        testSingleModule(drive, 0, "Right"),
        Commands.runOnce(drive::stop),
        Commands.print("--- DRIVE TEST: Swerve Module 1 (Front Right) ---"),
        testSingleModule(drive, 1, "Forward"),
        testSingleModule(drive, 1, "Backward"),
        testSingleModule(drive, 1, "Left"),
        testSingleModule(drive, 1, "Right"),
        Commands.runOnce(drive::stop),
        Commands.print("--- DRIVE TEST: Swerve Module 2 (Back Left) ---"),
        testSingleModule(drive, 2, "Forward"),
        testSingleModule(drive, 2, "Backward"),
        testSingleModule(drive, 2, "Left"),
        testSingleModule(drive, 2, "Right"),
        Commands.runOnce(drive::stop),
        Commands.print("--- DRIVE TEST: Swerve Module 3 (Back Right) ---"),
        testSingleModule(drive, 3, "Forward"),
        testSingleModule(drive, 3, "Backward"),
        testSingleModule(drive, 3, "Left"),
        testSingleModule(drive, 3, "Right"),
        Commands.runOnce(drive::stop),

        // INTAKE SUBSYSTEM TEST
        Commands.print("--- INTAKE TEST: Slapdown ---"),
        Commands.print("Slapdown: Moving DOWN"),
        Commands.runOnce(() -> intake.setSlapdownGoal(SlapdownGoal.DOWN), intake),
        Commands.waitUntil(intake::isSlapdownDown),
        Commands.waitSeconds(0.5),
        Commands.print("Slapdown: Moving UP"),
        Commands.runOnce(() -> intake.setSlapdownGoal(SlapdownGoal.UP), intake),
        Commands.waitUntil(intake::isSlapdownUp),
        Commands.waitSeconds(0.5),
        Commands.print("Slapdown: Moving DOWN again"),
        Commands.runOnce(() -> intake.setSlapdownGoal(SlapdownGoal.DOWN), intake),
        Commands.waitUntil(intake::isSlapdownDown),
        Commands.waitSeconds(0.5),
        Commands.print("Slapdown: JITTER test"),
        Commands.runOnce(() -> intake.setSlapdownGoal(SlapdownGoal.JITTER), intake),
        Commands.waitSeconds(2.0),
        Commands.runOnce(() -> intake.setSlapdownGoal(SlapdownGoal.UP), intake),
        Commands.waitUntil(intake::isSlapdownUp),
        Commands.print("--- INTAKE TEST: Roller ---"),
        Commands.print("Roller: INTAKE IN"),
        Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.INTAKE), intake),
        Commands.waitSeconds(1.5),
        Commands.print("Roller: STOP"),
        Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.STOP), intake),
        Commands.waitSeconds(0.5),
        Commands.print("Roller: OUTTAKE OUT"),
        Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.OUTTAKE), intake),
        Commands.waitSeconds(1.5),
        Commands.print("Roller: STOP"),
        Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.STOP), intake),
        Commands.waitSeconds(0.5),

        // INDEXER SUBSYSTEM TEST
        Commands.print("--- INDEXER TEST ---"),
        Commands.print("Indexer: FEED forward"),
        Commands.runOnce(() -> indexer.setGoal(IndexerGoal.FEED), indexer),
        Commands.waitSeconds(1.5),
        Commands.print("Indexer: STOP"),
        Commands.runOnce(() -> indexer.setGoal(IndexerGoal.STOP), indexer),
        Commands.waitSeconds(0.5),
        Commands.print("Indexer: REVERSE"),
        Commands.runOnce(() -> indexer.setGoal(IndexerGoal.REVERSE), indexer),
        Commands.waitSeconds(1.5),
        Commands.print("Indexer: STOP"),
        Commands.runOnce(() -> indexer.setGoal(IndexerGoal.STOP), indexer),
        Commands.waitSeconds(0.5),

        // SHOOTER SUBSYSTEM TEST
        Commands.print("--- SHOOTER TEST: Flywheel speeds RPM (2000.0, 2500.0, 3100.0) ---"),
        testFlywheelSpeeds(shooter, FLYWHEEL_TEST_SPEEDS_RPM),
        Commands.waitSeconds(0.5),
        Commands.print("--- SHOOTER TEST: Hood positions angles (10.0, 20.0, 35.0) ---"),
        testHoodPositions(shooter, HOOD_TEST_ANGLES_DEG),
        Commands.waitSeconds(0.5),
        Commands.print("--- SHOOTER TEST: Feeder ---"),
        Commands.print("Feeder: FEEDING"),
        Commands.runOnce(shooter::feedNote, shooter),
        Commands.waitSeconds(1.5),
        Commands.print("Feeder: STOP"),
        Commands.runOnce(shooter::stopFeeder, shooter),
        Commands.waitSeconds(0.5),
        Commands.print("Feeder: EJECTING"),
        Commands.runOnce(shooter::ejectNote, shooter),
        Commands.waitSeconds(1.5),
        Commands.print("Feeder: STOP"),
        Commands.runOnce(shooter::stopFeeder, shooter),
        Commands.waitSeconds(0.5),

        // DONE
        Commands.print("=== SYSTEM TEST COMPLETE ==="),
        Commands.runOnce( // stop evertyhing
            () -> {
              drive.stop();
              shooter.stop();
              shooter.stopFeeder();
              indexer.setGoal(IndexerGoal.STOP);
              intake.setSlapdownGoal(SlapdownGoal.UP);
              intake.setRollerGoal(RollerGoal.STOP);
            }));

    addRequirements(drive, intake, indexer, shooter);
  }

  /**
   * Test a single swerve module by driving it in the specified direction while all others stop.
   *
   * @param moduleIndex 0=FL, 1=FR, 2=BL, 3=BR
   * @param direction "Forward", "Backward", "Left", or "Right"
   */
  private static Command testSingleModule(Drive drive, int moduleIndex, String direction) {
    return Commands.run(
            () -> {
              double speed = 0.5 * drive.getMaxLinearSpeedMetersPerSec();
              Rotation2d angle;
              double driveSpeed;
              switch (direction) {
                case "Forward":
                  angle = Rotation2d.fromDegrees(0);
                  driveSpeed = speed;
                  break;
                case "Backward":
                  angle = Rotation2d.fromDegrees(180);
                  driveSpeed = speed;
                  break;
                case "Left":
                  angle = Rotation2d.fromDegrees(90);
                  driveSpeed = speed;
                  break;
                case "Right":
                  angle = Rotation2d.fromDegrees(-90);
                  driveSpeed = speed;
                  break;
                default:
                  angle = Rotation2d.kZero;
                  driveSpeed = 0;
              }
              drive.runSingleModule(moduleIndex, new SwerveModuleState(driveSpeed, angle));
              Logger.recordOutput("SystemTest/ModuleIndex", moduleIndex);
              Logger.recordOutput("SystemTest/DriveDirection", direction);
            },
            drive)
        .withTimeout(DURATION_SECONDS);
  }

  /** Test flywheel speeds by spinning up to each speed. */
  private static Command testFlywheelSpeeds(Shooter shooter, double[] speedsRPM) {
    Command[] commands = new Command[speedsRPM.length];
    for (int i = 0; i < speedsRPM.length; i++) {
      final double speed = speedsRPM[i];
      commands[i] =
          Commands.sequence(
              Commands.print("Flywheel: Spinning up to " + speed + " RPM"),
              Commands.runOnce(
                  () -> {
                    shooter.setFlywheelVelocity(speed);
                    Logger.recordOutput("SystemTest/FlywheelTestSpeed", speed);
                  },
                  shooter),
              Commands.waitSeconds(2.0),
              Commands.print("Flywheel: " + speed + " RPM achieved (or timeout)"),
              Commands.runOnce(shooter::stop, shooter),
              Commands.waitSeconds(0.5));
    }
    return Commands.sequence(commands);
  }

  /** Test hood positions by moving to each angle. */
  private static Command testHoodPositions(Shooter shooter, double[] anglesDeg) {
    Command[] commands = new Command[anglesDeg.length];
    for (int i = 0; i < anglesDeg.length; i++) {
      final double angleDeg = anglesDeg[i];
      final double angleRad = Units.degreesToRadians(angleDeg);
      commands[i] =
          Commands.sequence(
              Commands.print("Hood: Moving to " + angleDeg + " degrees"),
              Commands.runOnce(
                  () -> {
                    shooter.setHoodPosition(angleRad);
                    Logger.recordOutput("SystemTest/HoodTestAngleDeg", angleDeg);
                  },
                  shooter),
              Commands.waitSeconds(1.5),
              Commands.print("Hood: " + angleDeg + " degrees (or timeout)"),
              Commands.waitSeconds(0.5));
    }
    return Commands.sequence(commands);
  }
}
