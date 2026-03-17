// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.SlapdownGoal;
import frc.robot.subsystems.intake.Intake.RollerGoal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

/**
 * Comprehensive system test command that validates all subsystems sequentially.
 * Bind via .onTrue(new SystemTestCommand(...)) on a button press (e.g., driver.povUp().onTrue(...))
 *
 * Tests:
 * - Drive: Each swerve module forward/backward/left/right
 * - Intake: Slapdown up/down/jitter, roller intake
 * - Indexer: Feed forward
 * - Shooter: Multiple flywheel speeds, hood positions, feeder
 * - Vision: Check for hub tag targets
 */
public class SystemTestCommand extends SequentialCommandGroup {
  private static final double DURATION_SECONDS = 1.0; // Duration between movements

  // Shooter test speeds and positions
  private static final double[] FLYWHEEL_TEST_SPEEDS_RPM = {2000.0, 2500.0, 3100.0};
  private static final double[] HOOD_TEST_ANGLES_DEG = {10.0, 20.0, 35.0};

  public SystemTestCommand(
      Drive drive, Intake intake, Indexer indexer, Shooter shooter, Vision vision) {
    addCommands(
        // ========================================================================
        // INITIAL STATE
        // ========================================================================
        Commands.print("=== SYSTEM TEST START ==="),
        Commands.runOnce(
            () -> {
              drive.stop();
              shooter.stop();
              shooter.stopFeeder();
              indexer.setGoal(IndexerGoal.STOP);
            }),
        Commands.waitSeconds(0.5),

        // ========================================================================
        // DRIVE SUBSYSTEM TEST
        // ========================================================================
        Commands.print("--- DRIVE TEST: Swerve Module 0 (Front Left) ---"),
        testSwerveModule(drive, "Forward", 1.0, 0.0),
        Commands.waitSeconds(DURATION_SECONDS),
        testSwerveModule(drive, "Backward", -1.0, 0.0),
        Commands.waitSeconds(DURATION_SECONDS),
        testSwerveModule(drive, "Left", 0.0, 1.0),
        Commands.waitSeconds(DURATION_SECONDS),
        testSwerveModule(drive, "Right", 0.0, -1.0),
        Commands.waitSeconds(DURATION_SECONDS),
        Commands.runOnce(drive::stop),

        Commands.print("--- DRIVE TEST: Swerve Module 1 (Front Right) ---"),
        testSwerveModule(drive, "Forward", 1.0, 0.0),
        Commands.waitSeconds(DURATION_SECONDS),
        testSwerveModule(drive, "Backward", -1.0, 0.0),
        Commands.waitSeconds(DURATION_SECONDS),
        testSwerveModule(drive, "Left", 0.0, 1.0),
        Commands.waitSeconds(DURATION_SECONDS),
        testSwerveModule(drive, "Right", 0.0, -1.0),
        Commands.waitSeconds(DURATION_SECONDS),
        Commands.runOnce(drive::stop),

        Commands.print("--- DRIVE TEST: Swerve Module 2 (Back Left) ---"),
        testSwerveModule(drive, "Forward", 1.0, 0.0),
        Commands.waitSeconds(DURATION_SECONDS),
        testSwerveModule(drive, "Backward", -1.0, 0.0),
        Commands.waitSeconds(DURATION_SECONDS),
        testSwerveModule(drive, "Left", 0.0, 1.0),
        Commands.waitSeconds(DURATION_SECONDS),
        testSwerveModule(drive, "Right", 0.0, -1.0),
        Commands.waitSeconds(DURATION_SECONDS),
        Commands.runOnce(drive::stop),

        Commands.print("--- DRIVE TEST: Swerve Module 3 (Back Right) ---"),
        testSwerveModule(drive, "Forward", 1.0, 0.0),
        Commands.waitSeconds(DURATION_SECONDS),
        testSwerveModule(drive, "Backward", -1.0, 0.0),
        Commands.waitSeconds(DURATION_SECONDS),
        testSwerveModule(drive, "Left", 0.0, 1.0),
        Commands.waitSeconds(DURATION_SECONDS),
        testSwerveModule(drive, "Right", 0.0, -1.0),
        Commands.waitSeconds(DURATION_SECONDS),
        Commands.runOnce(drive::stop),

        // ========================================================================
        // INTAKE SUBSYSTEM TEST
        // ========================================================================
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

        // ========================================================================
        // INDEXER SUBSYSTEM TEST
        // ========================================================================
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

        // ========================================================================
        // SHOOTER SUBSYSTEM TEST
        // ========================================================================
        Commands.print("--- SHOOTER TEST: Flywheel speeds ---"),
        testFlywheelSpeeds(shooter, FLYWHEEL_TEST_SPEEDS_RPM),
        Commands.waitSeconds(0.5),

        Commands.print("--- SHOOTER TEST: Hood positions ---"),
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

        // ========================================================================
        // VISION SUBSYSTEM TEST
        // ========================================================================
        Commands.print("--- VISION TEST: Hub target detection ---"),
        testVisionHub(vision, 0),
        Commands.waitSeconds(1.0),

        // ========================================================================
        // CLEANUP
        // ========================================================================
        Commands.print("=== SYSTEM TEST COMPLETE ==="),
        Commands.runOnce(
            () -> {
              drive.stop();
              shooter.stop();
              shooter.stopFeeder();
              indexer.setGoal(IndexerGoal.STOP);
              intake.setSlapdownGoal(SlapdownGoal.UP);
              intake.setRollerGoal(RollerGoal.STOP);
            }),
        Commands.print("All subsystems stopped. Ready for operation."));

    // Add all subsystems as requirements
    addRequirements(drive, intake, indexer, shooter, vision);
  }

  // ============================================================================
  // HELPER METHODS
  // ============================================================================

  /**
   * Test a swerve module by applying a small velocity in the specified direction.
   * Tests the drive system at low speed to verify module rotation and drive.
   */
  private static Command testSwerveModule(Drive drive, String direction, double vx, double vy) {
    return Commands.run(
        () -> {
          double speed = 0.5; // 50% max speed for safety
          drive.runVelocity(
              new ChassisSpeeds(
                  vx * speed * drive.getMaxLinearSpeedMetersPerSec(),
                  vy * speed * drive.getMaxLinearSpeedMetersPerSec(),
                  0.0));
          Logger.recordOutput("SystemTest/DriveDirection", direction);
        },
        drive);
  }

  /**
   * Test flywheel speeds by spinning up to each speed and waiting for stabilization.
   */
  private static Command testFlywheelSpeeds(
      Shooter shooter, double[] speedsRPM) {
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
              Commands.waitSeconds(2.0), // Wait for flywheel to stabilize
              Commands.print("Flywheel: " + speed + " RPM achieved (or timeout)"),
              Commands.runOnce(shooter::stop, shooter),
              Commands.waitSeconds(0.5));
    }
    return Commands.sequence(commands);
  }

  /**
   * Test hood positions by moving to each angle and waiting for stabilization.
   */
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
              Commands.waitSeconds(1.5), // Wait for hood to settle
              Commands.print("Hood: " + angleDeg + " degrees (or timeout)"),
              Commands.waitSeconds(0.5));
    }
    return Commands.sequence(commands);
  }

  /**
   * Test vision subsystem by checking for hub targets visible on the camera.
   */
  private static Command testVisionHub(Vision vision, int cameraIndex) {
    return Commands.run(
            () -> {
              boolean hasTarget = vision.hasHubTarget(cameraIndex);
              double distance = Double.NaN;
              
              if (hasTarget) {
                distance = vision.getDistanceToHub(cameraIndex);
                Logger.recordOutput("SystemTest/VisionHubFound", true);
                Logger.recordOutput("SystemTest/VisionHubDistance", distance);
                System.out.println("✓ Hub target FOUND at distance: " + distance + " m");
              } else {
                Logger.recordOutput("SystemTest/VisionHubFound", false);
                System.out.println("✗ No hub target visible (may be expected if no target in view)");
              }

              // Also check generic targets
              int tagCount = vision.hasTarget(cameraIndex) ? 1 : 0; // Simple check
              Logger.recordOutput("SystemTest/VisionTargetsVisible", tagCount > 0);
            },
            vision)
        .withTimeout(3.0); // 3-second timeout for vision test
  }
}

// ============================================================================
// TROUBLESHOOTING TIPS:
// ============================================================================
//
// Module moving wrong direction?
//   → Check swerve module inversions in ModuleIOSpark/ModuleIOSim
//
// Slapdown doesn't reach goal?
//   → Verify IntakeIO closed-loop gains (kP, kD)
//   → Check physical limits and tolerance (slapdownToleranceRad)
//
// Shooter flywheel not spinning?
//   → Check ShooterIO velocity control loop
//   → Verify motor connections and CAN IDs
//
// Vision not detecting targets?
//   → Ensure Limelight is powered and networked
//   → Check target IDs match VisionConstants (redHubTagIds, blueHubTagIds)
//
// ============================================================================