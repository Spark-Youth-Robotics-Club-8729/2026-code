// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKrakenX60;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final Intake intake;
  private final Indexer indexer;

  // Controllers
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard
  private final LoggedDashboardChooser<Command> autoChooser;

  // TODO: Set to the actual field position of your shoot target (e.g. speaker center)
  private static final Translation2d SHOOT_TARGET_POSITION = new Translation2d(0.0, 5.55);

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        // Limelight cameras — pass rotation supplier for MegaTag 2
        // camera0Name / camera1Name must match names in the Limelight web UI
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, () -> drive.getRotation()),
                new VisionIOLimelight(camera1Name, () -> drive.getRotation()));

        shooter = new Shooter(new ShooterIOKrakenX60());
        intake = new Intake(new IntakeIOHardware());
        indexer = new Indexer(new IndexerIOSparkMax());
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        // PhotonVision sim still works great for simulation even though real robot uses Limelight
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        shooter = new Shooter(new ShooterIOSim());
        intake = new Intake(new IntakeIOSim());
        indexer = new Indexer(new IndexerIOSim());
        break;

      default:
        // Replay — no-op IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        shooter = new Shooter(new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        break;
    }

    ShotCalculator.initialize(drive::getPose, SHOOT_TARGET_POSITION);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Add every .path file as a standalone auto option
    String[] pathNames = {
      // Starting position paths
      "BlueS1toN1",
      "BlueS1toN2",
      "BlueS1toN3",
      "BlueS1toN4",
      "BlueS1toN5",
      "BlueS1toN6",
      "BlueS1toT",
      "BlueS1toO",
      "BlueS2toN1",
      "BlueS2toN2",
      "BlueS2toN3",
      "BlueS2toN4",
      "BlueS2toN5",
      "BlueS2toN6",
      "BlueS2toT",
      "BlueS2toO",
      "BlueS2toD",
      "BlueS3toN1",
      "BlueS3toN2",
      "BlueS3toN3",
      "BlueS3toN4",
      "BlueS3toN5",
      "BlueS3toN6",
      "BlueS3toT",
      "BlueS3toO",
      "BlueS3toD",
      "BlueS4toN1",
      "BlueS4toN2",
      "BlueS4toN3",
      "BlueS4toN4",
      "BlueS4toN5",
      "BlueS4toN6",
      "BlueS4toT",
      "BlueS4toO",
      "BlueS4toD",
      // Note-to-target paths
      "BlueN1toTthorughT1",
      "BlueN1toTthorughB1",
      "BlueN1toHthroughT1",
      "BlueN1toHthroughB1",
      "BlueN2toTthorughT1",
      "BlueN2toTthorughB1",
      "BlueN2toHthroughT1",
      "BlueN2toHthroughB1",
      "BlueN3toTthorughT1",
      "BlueN3toTthorughT2",
      "BlueN3toTthorughB1",
      "BlueN3toTthorughB2",
      "BlueN3toHthroughT1",
      "BlueN3toHthroughB2",
      "BlueN4toTthorughT2",
      "BlueN4toHthroughT2",
      "BlueN4toHthroughB2",
      "BlueN5toTthorughT2",
      "BlueN5toTthorughB2",
      "BlueN5toHthroughT2",
      "BlueN5toHthroughB2",
      "BlueN6toTthorughT2",
      "BlueN6toTthorughB2",
      "BlueN6toHthroughT2",
      "BlueN6toHthroughB2",
      // Hopper/other paths
      "BlueHtoT",
      "BlueHtoO",
      "BlueHtoD",
      "BlueHtoN1throughT1",
      "BlueHtoN1throughB1",
      "BlueHtoN2throughT1",
      "BlueHtoN2throughB1",
      "BlueHtoN3throughT2",
      "BlueHtoN3ThroughT1",
      "BlueHtoN3throughB2",
      "BlueHtoN3throughB1",
      "BlueHtoN4throughT2",
      "BlueHtoN4throughT1",
      "BlueHtoN4throughB2",
      "BlueHtoN4throughB1",
      "BlueHtoN5throughT2",
      "BlueHtoN5throughB2",
      "BlueHtoN6throughT2",
      "BlueHtoN6throughB2",
      // Outpost paths
      "BlueOtoT",
      "BlueOtoH",
      "BlueOtoS1",
      "BlueOtoS2",
      "BlueOtoS3",
      "BlueOtoS4",
      // D paths
      "BlueDtoT",
      "BlueDtoH",
      "BlueDtoS1",
      "BlueDtoS2",
      "BlueDtoS3",
      "BlueDtoS4",
      // NBlue special path
      "NBlueN4toTthorughB2"
    };
    for (String pathName : pathNames) {
      try {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        autoChooser.addOption("Path: " + pathName, AutoBuilder.followPath(path));
      } catch (Exception e) {
        System.err.println("Failed to load path: " + pathName);
      }
    }

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // -------------------------------------------------------------------------
    // DRIVER (port 0)
    // -------------------------------------------------------------------------

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // A — lock heading to 0°
    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> Rotation2d.kZero));

    // B — reset gyro to 0°
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // X — X-pattern brake
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Y — auto-aim to best Limelight target while driving
    driver
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> drive.getRotation().plus(vision.getTargetX(0))));

    // -------------------------------------------------------------------------
    // OPERATOR (port 1)
    //
    // Button layout:
    //   Right Trigger — EVERYTHING: aim + spin flywheels + green feeder + indexer (when ready)
    //   Left Trigger  — AIM ONLY: spin flywheels + move hood (no feeding)
    //   Y             — GREEN FEEDER WHEELS only (force-feed, bypasses ready check)
    //   Right Bumper  — INDEXER only (feed toward shooter)
    //   Left Bumper   — INTAKE roller only
    //   B             — INTAKE outtake
    //   POV Up        — SLAPDOWN UP (raise arm)
    //   POV Down      — SLAPDOWN DOWN (lower arm)
    //   Left Stick    — TEST flywheels at default speed
    // -------------------------------------------------------------------------

    // Right Trigger — full shot sequence: aim + spin + green feeder + indexer when ready
    operator
        .rightTrigger(0.5)
        .whileTrue(
            Commands.run(
                    () -> {
                      var params = ShotCalculator.getInstance().calculate();
                      shooter.setHoodPosition(params.hoodAngleRad());
                      shooter.setFlywheelVelocity(params.flywheelSpeedRPM());
                      if (params.isValid() && shooter.isReadyToShoot()) {
                        shooter.feedNote();
                      } else {
                        shooter.stopFeeder();
                      }
                    },
                    shooter)
                .alongWith(
                    Commands.run(
                        () -> {
                          if (shooter.isReadyToShoot()) {
                            indexer.feed();
                          } else {
                            indexer.stop();
                          }
                        },
                        indexer))
                .finallyDo(
                    () -> {
                      shooter.stop();
                      indexer.stop();
                    }));

    // Left Trigger — aim only: spin flywheels + move hood (no feeding)
    operator
        .leftTrigger(0.5)
        .whileTrue(
            Commands.run(
                    () -> {
                      var params = ShotCalculator.getInstance().calculate();
                      shooter.setHoodPosition(params.hoodAngleRad());
                      shooter.setFlywheelVelocity(params.flywheelSpeedRPM());
                    },
                    shooter)
                .finallyDo(shooter::stop));

    // Y — green feeder wheels only (force-feed for testing, does NOT interrupt flywheel command)
    operator
        .y()
        .whileTrue(
            Commands.startEnd(shooter::feedNote, shooter::stopFeeder, shooter));

    // Right Bumper — indexer only
    operator.rightBumper().whileTrue(indexer.feedCommand());

    // Left Bumper — intake roller only
    operator.leftBumper().whileTrue(intake.intakeCommand());

    // B — outtake from intake roller
    operator.b().whileTrue(intake.outtakeCommand());

    // POV Down — lower slapdown arm
    operator.povDown().onTrue(intake.slapdownDownCommand());

    // POV Up — raise slapdown arm
    operator.povUp().onTrue(intake.retractCommand());

    // Left Stick — test flywheels at default speed
    operator
        .leftStick()
        .whileTrue(
            Commands.startEnd(
                () ->
                    shooter.setFlywheelVelocity(
                        frc.robot.subsystems.shooter.ShooterConstants.defaultFlywheelSpeedRPM),
                shooter::stop,
                shooter));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Drive getDrive() {
    return drive;
  }

  public Vision getVision() {
    return vision;
  }

  public Shooter getShooter() {
    return shooter;
  }

  public Intake getIntake() {
    return intake;
  }

  public Indexer getIndexer() {
    return indexer;
  }
}