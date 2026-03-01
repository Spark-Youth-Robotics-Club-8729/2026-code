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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LimelightAimAndRangeCommand;
import frc.robot.commands.LimelightAimCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKrakenX60;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.LimelightDistanceEstimator;
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

  // Distance estimator — TODO: fill in real measurements before competition
  private final LimelightDistanceEstimator distanceEstimator;

  // Manual preset shooting positions for bench/garage testing
  // { hoodAngleRad, flywheelRPM }
  private static final double[][] SHOOT_PRESETS = {
    { Units.degreesToRadians(15.0), 2000.0 }, // Preset 0: close
    { Units.degreesToRadians(30.0), 3000.0 }, // Preset 1: mid
    { Units.degreesToRadians(45.0), 4000.0 }, // Preset 2: far
  };
  private int shootPresetIndex = 0;

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

        // Single Limelight 4 — pass rotation supplier for MegaTag 2 and IMU assist
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));

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

        // In sim there is no Limelight hardware — use a no-op VisionIO
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});

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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        shooter = new Shooter(new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        break;
    }

    // Alliance-aware hub position supplier — ShotCalculator.getAllianceHubPosition() handles
    // the per-alliance lookup, but we still need an initial Translation2d for the constructor.
    // We pass a lambda so the pose supplier always reads the latest estimated position,
    // and use the blue hub as the default (will be corrected once DS connects).
    ShotCalculator.initialize(
        drive::getPose,
        drive::getChassisSpeeds,
        VisionConstants
            .BLUE_HUB_POSITION); // default; getAllianceHubPosition() overrides at runtime

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

    // Distance estimator (only meaningful on real hardware; values are placeholders)
    distanceEstimator =
        new LimelightDistanceEstimator(
            vision, 0, 20.0, // TODO: camera height above floor (inches)
            60.0, // TODO: target height above floor (inches)
            25.0); // TODO: camera mount angle above horizontal (degrees)

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // -------------------------------------------------------------------------
    // DRIVER (port 0)
    // -------------------------------------------------------------------------

    drive.setDefaultCommand(
        DriveCommands.joystickDriveRobotRelative(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> Rotation2d.kZero));

    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    driver
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> drive.getRotation().plus(vision.getTargetX(0))));

    // Driver LEFT BUMPER — Limelight proportional aim + driver translation
    driver
        .leftBumper()
        .whileTrue(
            new LimelightAimCommand(
                drive, vision, 0, () -> -driver.getLeftY(), () -> -driver.getLeftX()));

    // Driver RIGHT BUMPER — Limelight aim + auto range (no translation)
    driver.rightBumper().whileTrue(new LimelightAimAndRangeCommand(drive, vision, 0));

    // -------------------------------------------------------------------------
    // OPERATOR (port 1)
    //
    // Right Trigger — EVERYTHING: flywheels + hood + feeder + indexer (when ready)
    // Left Trigger  — AIM: flywheels + hood only (no feeding)
    // Y             — FEEDER WHEELS only
    // Right Bumper  — INDEXER only
    // Left Bumper   — INTAKE roller only
    // B             — INTAKE outtake
    // X             — SLAPDOWN DOWN
    // Right Stick   — SLAPDOWN UP / retract
    // POV Down      — SLAPDOWN DOWN
    // POV Up        — SLAPDOWN UP
    // POV Left      — cycle shoot preset DOWN
    // POV Right     — cycle shoot preset UP
    // Left Stick    — TEST flywheels at default speed
    // A             — Full auto-shoot
    // -------------------------------------------------------------------------

    // Right Trigger — EVERYTHING when ready (uses manual preset)
    operator
        .rightTrigger(0.5)
        .whileTrue(
            Commands.run(
                    () -> {
                      shooter.setHoodPosition(SHOOT_PRESETS[shootPresetIndex][0]);
                      shooter.setFlywheelVelocity(SHOOT_PRESETS[shootPresetIndex][1]);
                      if (shooter.isReadyToShoot()) {
                        shooter.feedNote();
                        // Don't call stopFeeder() in the else branch —
                        // doing so would stomp on the Y-button feeder command every loop.
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
                      shooter.stopFeeder();
                      indexer.stop();
                    }));

    // Left Trigger — AIM only: flywheels + hood, no feeding (uses manual preset)
    operator
        .leftTrigger(0.5)
        .whileTrue(
            Commands.run(
                    () -> {
                      shooter.setHoodPosition(SHOOT_PRESETS[shootPresetIndex][0]);
                      shooter.setFlywheelVelocity(SHOOT_PRESETS[shootPresetIndex][1]);
                    },
                    shooter)
                .finallyDo(
                    () -> {
                      shooter.stop();
                      // Do NOT call stopFeeder() — left trigger never owned the feeder
                    }));

    // Y — FEEDER WHEELS only (no subsystem requirement — coexists with flywheel commands)
    operator
        .y()
        .whileTrue(
            Commands.startEnd(
                shooter::feedNote,
                shooter::stopFeeder
            ));

    // Right Bumper — INTAKE IN
    operator.rightBumper().whileTrue(intake.intakeCommand());

    // Left Bumper — INTAKE OUT
    operator.leftBumper().whileTrue(intake.outtakeCommand());

    // B — INDEXER IN
    operator.b().whileTrue(indexer.feedCommand());

    // POV Down — SLAPDOWN DOWN
    operator.povDown().onTrue(intake.slapdownDownCommand());

    // POV Up — SLAPDOWN UP
    operator.povUp().onTrue(intake.retractCommand());

    // POV Left — cycle shoot preset DOWN
    operator
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shootPresetIndex = Math.max(0, shootPresetIndex - 1);
                  System.out.println(
                      "Shoot preset: "
                          + shootPresetIndex
                          + " | Hood: "
                          + Math.toDegrees(SHOOT_PRESETS[shootPresetIndex][0])
                          + "deg | RPM: "
                          + SHOOT_PRESETS[shootPresetIndex][1]);
                }));

    // POV Right — cycle shoot preset UP
    operator
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shootPresetIndex = Math.min(SHOOT_PRESETS.length - 1, shootPresetIndex + 1);
                  System.out.println(
                      "Shoot preset: "
                          + shootPresetIndex
                          + " | Hood: "
                          + Math.toDegrees(SHOOT_PRESETS[shootPresetIndex][0])
                          + "deg | RPM: "
                          + SHOOT_PRESETS[shootPresetIndex][1]);
                }));

    // Left Stick — TEST flywheels at default speed
    operator
        .leftStick()
        .whileTrue(
            Commands.startEnd(
                () ->
                    shooter.setFlywheelVelocity(
                        frc.robot.subsystems.shooter.ShooterConstants.defaultFlywheelSpeedRPM),
                shooter::stop,
                shooter));

    // Operator A — full auto-shoot (visual TX aim + spin up + feed)
    operator.a().whileTrue(new AutoShootCommand(drive, shooter, indexer, vision, 0));
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

  public LimelightDistanceEstimator getDistanceEstimator() {
    return distanceEstimator;
  }
}
