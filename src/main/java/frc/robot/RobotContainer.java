// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.shooter.ShooterConstants.defaultFlywheelSpeedRPM;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LimelightAimAndRangeCommand;
import frc.robot.commands.LimelightAimCommand;
import frc.robot.commands.ManualAuto;
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
import frc.robot.subsystems.shooter.ShooterConstants;
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

  // (SHOOT_PRESETS and shootPresetIndex removed — now using ShotCalculator)

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

    registerNamedCommands();

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
      "BlueDtoS3",
      "BlueDtoS4",
      // NBlue special path
      "NBlueN4toTthorughB2",
      "Elims1path"
    };
    for (String pathName : pathNames) {
      try {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        autoChooser.addOption("Path: " + pathName, AutoBuilder.followPath(path));
      } catch (Exception e) {
        System.err.println("Failed to load path: " + pathName);
      }
    }

    autoChooser.addOption("Manual Auto", ManualAuto.simpleAuto(drive));

    // Distance estimator (only meaningful on real hardware; values are placeholders)
    distanceEstimator =
        new LimelightDistanceEstimator(
            vision, 0, 20.0, // TODO: camera height above floor (inches)
            60.0, // TODO: target height above floor (inches)
            25.0); // TODO: camera mount angle above horizontal (degrees)

    configureButtonBindings();
  }

  private void registerNamedCommands() {
    // Vision-assisted shooting/aiming
    NamedCommands.registerCommand(
        "AutoShootCommand", new AutoShootCommand(drive, shooter, indexer, vision, 0));
    NamedCommands.registerCommand(
        "AutoShootSpinUpWindow", autoShootWindow(2.0, false)); // TODO: Fix time
    NamedCommands.registerCommand(
        "AutoShootFireWindow", autoShootWindow(4.0, true)); // TODO: Fix time
    NamedCommands.registerCommand(
        "LimelightAim", new LimelightAimCommand(drive, vision, 0, () -> 0.0, () -> 0.0));
    NamedCommands.registerCommand(
        "LimelightAimAndRange", new LimelightAimAndRangeCommand(drive, vision, 0));

    // Shooter orchestration
    NamedCommands.registerCommand(
        "ShooterSpinUpDefault",
        Commands.startEnd(
            () -> {
              shooter.setHoodPosition(ShooterConstants.hoodMinAngleRad);
              shooter.setFlywheelVelocity(defaultFlywheelSpeedRPM);
            },
            () -> shooter.stop(),
            shooter));
    NamedCommands.registerCommand(
        "ShooterFeed",
        Commands.startEnd(
            () -> {
              shooter.feedNote();
              indexer.feed();
            },
            () -> {
              shooter.stopFeeder();
              indexer.stop();
            },
            shooter,
            indexer));
    NamedCommands.registerCommand(
        "ShooterStop",
        Commands.runOnce(
            () -> {
              shooter.stop();
              shooter.stopFeeder();
              indexer.stop();
            },
            shooter,
            indexer));

    // Intake routines (multiple aliases for flexibility)
    NamedCommands.registerCommand("SlapdownAndIntakeCommand", intake.slapdownAndIntakeCommand());
    NamedCommands.registerCommand("StartIntakeSlapdown", intake.slapdownAndIntakeCommand());
    NamedCommands.registerCommand("IntakeIN", intake.intakeCommand());
    NamedCommands.registerCommand("IntakeIn", intake.intakeCommand());
    NamedCommands.registerCommand("IntakeOUT", intake.outtakeCommand());
    NamedCommands.registerCommand("IntakeOut", intake.outtakeCommand());
    NamedCommands.registerCommand("IntakeRetract", intake.retractCommand());
    NamedCommands.registerCommand("RetractIntake", intake.retractCommand());
    NamedCommands.registerCommand("IntakeSlapdownDownOnly", intake.slapdownDownCommand());
    NamedCommands.registerCommand("IntakeJitter", intake.jitterCommand());
    NamedCommands.registerCommand(
        "IntakeToggleSlapdown", Commands.runOnce(intake::toggleSlapdown, intake));

    // Indexer helpers
    NamedCommands.registerCommand("IndexerFeed", indexer.feedCommand());
    NamedCommands.registerCommand("IndexerReverse", indexer.reverseCommand());
    NamedCommands.registerCommand("IndexerStop", indexer.stopCommand());

    // Aim-only action (no translation) with a timeout to avoid running forever
    NamedCommands.registerCommand(
        "AutoAimHub",
        new LimelightAimCommand(drive, vision, 0, () -> 0.0, () -> 0.0).withTimeout(2.0));

    // Jitter + shoot / no-jitter shoot routines
    NamedCommands.registerCommand("JitterShoot10s", buildJitterShootCommand());
    NamedCommands.registerCommand("ShootNoJitter10s", buildShootNoJitterCommand());

    // Stop everything and retract intake
    NamedCommands.registerCommand(
        "StopIntakeAndShooter",
        Commands.runOnce(
                () -> {
                  intake.retractCommand().schedule();
                  indexer.stop();
                  shooter.stopFeeder();
                  shooter.stop();
                },
                intake,
                indexer,
                shooter)
            .ignoringDisable(true));
  }

  private Command autoShootWindow(double durationSeconds, boolean withJitter) {
    return new ProxyCommand(
        () -> {
          Command autoShoot = new AutoShootCommand(drive, shooter, indexer, vision, 0);
          if (withJitter) {
            autoShoot = autoShoot.alongWith(intake.jitterCommand());
          }
          return Commands.waitSeconds(durationSeconds).deadlineWith(autoShoot);
        });
  }

  private Command buildJitterShootCommand() {
    Command jitter = intake.jitterCommand();

    Command shootWhileReady =
        Commands.run(
                () -> {
                  shooter.setFlywheelVelocity(defaultFlywheelSpeedRPM);
                  shooter.setHoodPosition(Units.degreesToRadians(10.0));
                  if (shooter.isReadyToShoot()) {
                    Commands.waitSeconds(2.0);
                    indexer.feed();
                    shooter.feedNote();
                  } else {
                    shooter.stopFeeder();
                    indexer.stop();
                  }
                },
                shooter,
                indexer)
            .finallyDo(
                () -> {
                  indexer.stop();
                  shooter.stopFeeder();
                  shooter.stop();
                });

    return Commands.parallel(jitter, shootWhileReady).withTimeout(10.0);
  }

  private Command buildShootNoJitterCommand() {
    // Make sure intake is up and roller stopped before shooting
    Command prep =
        Commands.runOnce(
                () -> {
                  intake.retractCommand().schedule();
                  intake.setRollerGoal(frc.robot.subsystems.intake.Intake.RollerGoal.STOP);
                },
                intake)
            .ignoringDisable(true);

    Command shootWhileReady =
        Commands.run(
                () -> {
                  shooter.setFlywheelVelocity(defaultFlywheelSpeedRPM);

                  if (shooter.isReadyToShoot()) {
                    indexer.feed();
                    shooter.feedNote();
                  } else {
                    shooter.stopFeeder();
                    indexer.stop();
                  }
                },
                shooter,
                indexer)
            .finallyDo(
                () -> {
                  indexer.stop();
                  shooter.stopFeeder();
                  shooter.stop();
                });

    return prep.andThen(shootWhileReady.withTimeout(10.0));
  }

  private void configureButtonBindings() {
    // -------------------------------------------------------------------------
    // DRIVER (port 0)
    //
    // Left Stick    — FIELD-RELATIVE translation (X and Y)
    // Right Stick   — Rotation (Omega)
    // A             — HOLD to snap/drive at 0 degrees (facing forward)    -- kinda works
    // B             — RESET GYRO to current heading (sets rotation to 0)   -- test pls
    // X             — X-BRAKE (lock wheels in X-pattern to resist pushing)      -- works pretty
    // sure
    // Y             — HOLD to Limelight aim (Auto-rotate to target) while driving  -- doesnt work
    // Left Bumper   — HOLD for Proportional Limelight Aiming + Manual Translation  -- test pls
    // Right Bumper  — HOLD for Limelight Aiming + Automatic Range/Distance logic  -- test pls
    // POV Down      — PRESS to set hood down to its minimum resting angle
    // -------------------------------------------------------------------------

    // WORKS NOW (intake is frontside): 
    drive.setDefaultCommand(
        DriveCommands.joystickDrive( // for robot relative, do this: joystickDriveRobotRelative
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> Rotation2d.kZero));

    // driver
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //                 drive)
    //             .ignoringDisable(true));

    driver
        .b()
        .whileTrue(
            Commands.run(
                () -> {
                  drive.zeroGyro(); // not working
                }));

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

    // Driver POV Down — Manually push the hood down to its minimum resting angle
    driver
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.setHoodPosition(
                      frc.robot.subsystems.shooter.ShooterConstants.hoodMinAngleRad);
                },
                shooter));

    // -------------------------------------------------------------------------
    // OPERATOR (port 1)
    //
    // Right Trigger  — HOLD to spin up flywheels + hood and AUTO-FEED once ready
    // Left Trigger   — HOLD for high-arcing "Clearance" shot to pass fuel
    // Y              — HOLD to run FEEDER wheels in (manual intake to shooter)
    // X              — HOLD to EJECT (runs feeder and flywheels in reverse)
    // Right Bumper   — HOLD to run INTAKE wheels in
    // Left Bumper    — HOLD to run INTAKE wheels out
    // B              — HOLD to run INDEXER wheels in
    // A              — HOLD for FULL AUTO-SHOOT (Vision aim + Spin + Feed)
    // if it gets full routine done
    // POV Down       — PRESS to toggle intake SLAPDOWN (up/down)
    // POV Up         — HOLD to JITTER/agitate balls in intake
    // POV Left       — PRESS to nudge hood angle DOWN (-1 degree)
    // POV Right      — PRESS to nudge hood angle UP (+1 degree)
    // Left Stick     — HOLD to test flywheels at default 3000 RPM
    // Right Stick    — HOLD for HARDCODED TRENCH SHOT (Preset angle/RPM)   -- havent tested yet,
    // adjust values pls
    // -------------------------------------------------------------------------

    // Right Trigger — shoot: spin up flywheels + hood, then feed once both are at target.
    // Hood and flywheel targets come from ShotCalculator using nearest AprilTag distance.
    // VERSION 0: (initial version with indexer.feed() inside the if statement)
    double[] lastValidDist = {ShooterConstants.hoodMinAngleRad};
    operator
        .rightTrigger(0.5)
        .whileTrue(
            Commands.run(
                    () -> {
                      // --- Vision-based shot parameters ---
                      double rawDist = vision.getNearestTagDistance(0);
                      if (!Double.isNaN(rawDist) && rawDist > 0.1) {
                        lastValidDist[0] = rawDist;
                      }
                      double dist =
                          lastValidDist[
                              0]; // assuming blocked tag ID code works, then the hood won't jitter
                      // up & down
                      double hoodAngle;
                      double flywheelRPM;
                      if (vision.hasTarget(0) && !Double.isNaN(dist) && dist > 0.1) {
                        var params =
                            ShotCalculator.getInstance()
                                .calculateFromDistance(dist, drive.getPose().getRotation());
                        hoodAngle = params.hoodAngleRad();
                        flywheelRPM =
                            params.flywheelSpeedRPM()
                                + 200; // TEMPORARY INCREASE ----- PLEASE FIX SHOT CALCULATOR :sob
                      } else {
                        // No tag — safe default (close range)
                        hoodAngle = ShooterConstants.hoodMinAngleRad;
                        flywheelRPM = ShooterConstants.defaultFlywheelSpeedRPM;
                      }

                      // Always spin up hood and flywheels
                      shooter.setHoodPosition(hoodAngle);
                      shooter.setFlywheelVelocity(flywheelRPM);

                      // Debug prints
                      System.out.println("dist: " + dist);
                      // System.out.println("hoodAngleCalculated: " +
                      // Units.radiansToDegrees(hoodAngle) + " deg");
                      System.out.println("calculatedFlywheelRPM: " + flywheelRPM);
                      // System.out.println("flywheelSpeed: " + shooter.getLeftFlywheelVelocity());
                      // System.out.println()"curHoodPosition: " +
                      // Units.radiansToDegrees(shooter.getHoodPosition()) + " deg");
                      System.out.println(
                          "deltaFlywheelSpeed: "
                              + (flywheelRPM - shooter.getLeftFlywheelVelocity()));
                      System.out.println(
                          "deltaHoodPosition(0=good): "
                              + (Units.radiansToDegrees(hoodAngle)
                                  - Units.radiansToDegrees(shooter.getHoodPosition()))
                              + " deg");

                      // Only feed once flywheels are at speed AND hood is at position
                      if (shooter.areFlywheelsAtSpeed() && shooter.isHoodAtPosition()) {
                        indexer.feed();
                        shooter.feedNote();
                      } else {
                        shooter.stopFeeder();
                        indexer.stop(); // keep or leave it????? idk
                      }
                    },
                    shooter,
                    indexer)
                .finallyDo(
                    () -> {
                      shooter.stop();
                      shooter.stopFeeder();
                      indexer.stop();
                    }));

    // VERSION 1: (put indexer.feed() outside of the if statement)
    /*
    double[] lastValidDist = {ShooterConstants.hoodMinAngleRad};
    operator
        .rightTrigger(0.5)
        .whileTrue(
              Commands.run(
                    () -> {
                      // --- Vision-based shot parameters ---
                      double rawDist = vision.getNearestTagDistance(0);
                      if (!Double.isNaN(rawDist) && rawDist > 0.1) {
                        lastValidDist[0] = rawDist;
                      }
                      double dist = lastValidDist[0];           // assuming blocked tag ID code works, then the hood won't jitter up & down
                      double hoodAngle;
                      double flywheelRPM;
                      if (vision.hasTarget(0) && !Double.isNaN(dist) && dist > 0.1) {
                        var params =
                            ShotCalculator.getInstance()
                                .calculateFromDistance(dist, drive.getPose().getRotation());
                        hoodAngle = params.hoodAngleRad();
                        flywheelRPM = params.flywheelSpeedRPM() + 500;    // TEMPORARY INCREASE ----- PLEASE FIX SHOT CALCULATOR :sob
                      } else {
                        // No tag — safe default (close range)
                        hoodAngle = ShooterConstants.hoodMinAngleRad;
                        flywheelRPM = ShooterConstants.defaultFlywheelSpeedRPM;
                      }

                      // Always spin up hood and flywheels
                      shooter.setHoodPosition(hoodAngle);
                      shooter.setFlywheelVelocity(flywheelRPM);

                      // Debug prints
                      System.out.println("dist: " + dist);
                      // System.out.println("hoodAngleCalculated: " +
                      // Units.radiansToDegrees(hoodAngle) + " deg");
                      System.out.println("calculatedFlywheelRPM: " + flywheelRPM);
                      // System.out.println("flywheelSpeed: " + shooter.getLeftFlywheelVelocity());
                      // System.out.println()"curHoodPosition: " +
                      // Units.radiansToDegrees(shooter.getHoodPosition()) + " deg");
                      System.out.println(
                          "deltaFlywheelSpeed: "
                              + (flywheelRPM - shooter.getLeftFlywheelVelocity()));
                      System.out.println(
                          "deltaHoodPosition(0=good): "
                              + (Units.radiansToDegrees(hoodAngle)
                                  - Units.radiansToDegrees(shooter.getHoodPosition()))
                              + " deg");

                      indexer.feed();   // simply just put outside of the if statement, so its always ran

                      // Only feed once flywheels are at speed AND hood is at position
                      if (shooter.areFlywheelsAtSpeed() && shooter.isHoodAtPosition()) {
                        shooter.feedNote();
                      } else {
                        shooter.stopFeeder();
                        //indexer.stop();  // keep or leave it????? idk
                      }
                    },
                    shooter,
                    indexer)
                .finallyDo(
                    () -> {
                      shooter.stop();
                      shooter.stopFeeder();
                      indexer.stop();
                    })); */

    // VERSION 2: (used feedCommand() and Commands.parallel)
    /*
    double[] lastValidDist = {ShooterConstants.hoodMinAngleRad};
    operator
        .rightTrigger(0.5)
        .whileTrue(
            Commands.parallel(
              indexer.feedCommand(),

              Commands.run(
                    () -> {
                      // --- Vision-based shot parameters ---
                      double rawDist = vision.getNearestTagDistance(0);
                      if (!Double.isNaN(rawDist) && rawDist > 0.1) {
                        lastValidDist[0] = rawDist;
                      }
                      double dist = lastValidDist[0];           // assuming blocked tag ID code works, then the hood won't jitter up & down
                      double hoodAngle;
                      double flywheelRPM;
                      if (vision.hasTarget(0) && !Double.isNaN(dist) && dist > 0.1) {
                        var params =
                            ShotCalculator.getInstance()
                                .calculateFromDistance(dist, drive.getPose().getRotation());
                        hoodAngle = params.hoodAngleRad();
                        flywheelRPM = params.flywheelSpeedRPM() + 500;    // TEMPORARY INCREASE ----- PLEASE FIX SHOT CALCULATOR :sob
                      } else {
                        // No tag — safe default (close range)
                        hoodAngle = ShooterConstants.hoodMinAngleRad;
                        flywheelRPM = ShooterConstants.defaultFlywheelSpeedRPM;
                      }

                      // Always spin up hood and flywheels
                      shooter.setHoodPosition(hoodAngle);
                      shooter.setFlywheelVelocity(flywheelRPM);

                      // Debug prints
                      System.out.println("dist: " + dist);
                      // System.out.println("hoodAngleCalculated: " +
                      // Units.radiansToDegrees(hoodAngle) + " deg");
                      System.out.println("calculatedFlywheelRPM: " + flywheelRPM);
                      // System.out.println("flywheelSpeed: " + shooter.getLeftFlywheelVelocity());
                      // System.out.println()"curHoodPosition: " +
                      // Units.radiansToDegrees(shooter.getHoodPosition()) + " deg");
                      System.out.println(
                          "deltaFlywheelSpeed: "
                              + (flywheelRPM - shooter.getLeftFlywheelVelocity()));
                      System.out.println(
                          "deltaHoodPosition(0=good): "
                              + (Units.radiansToDegrees(hoodAngle)
                                  - Units.radiansToDegrees(shooter.getHoodPosition()))
                              + " deg");

                      // Only feed once flywheels are at speed AND hood is at position
                      if (shooter.areFlywheelsAtSpeed() && shooter.isHoodAtPosition()) {
                        shooter.feedNote();
                      } else {
                        shooter.stopFeeder();
                      }
                    },
                    shooter)
                .finallyDo(
                    () -> {
                      shooter.stop();
                      shooter.stopFeeder();
                      indexer.stop();
                    }))); */

    // Left Trigger — High Arcing "Neutral Zone to Alliance Zone" Shot (essentially to pass the fuel
    // to our side)
    operator
        .leftTrigger(0.5)
        .whileTrue(
            Commands.run(
                    () -> {
                      // Use max hood angle for high arc and high velocity for distance
                      double highArcHoodAngle =
                          ShooterConstants.hoodMaxAngleRad
                              - Units.degreesToRadians(45.0); // Adjust this!!!!
                      double highVelocityRPM =
                          ShooterConstants.maxFlywheelSpeedRPM - 5000; // Adjust this!!!!

                      shooter.setHoodPosition(highArcHoodAngle);
                      shooter.setFlywheelVelocity(highVelocityRPM);

                      // Feed once ready
                      if (shooter.areFlywheelsAtSpeed() && shooter.isHoodAtPosition()) {
                        shooter.feedNote();
                        indexer.feed();
                      } else {
                        shooter.stopFeeder();
                        indexer.stop();
                      }
                    },
                    shooter,
                    indexer)
                .finallyDo(
                    () -> {
                      shooter.stop();
                      shooter.stopFeeder();
                      indexer.stop();
                    }));

    // Y — FEEDER WHEELS only in (no subsystem requirement)
    operator.y().whileTrue(Commands.startEnd(shooter::feedNote, shooter::stopFeeder));

    // X — FEEDER WHEELS and FLYWHEELS out
    operator
        .x()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  shooter.ejectNote();
                  shooter.setFlywheelVelocity(-defaultFlywheelSpeedRPM); // Adjust RPM if needed
                },
                () -> {
                  shooter.stopFeeder();
                  shooter.stop();
                },
                shooter));
    operator
        .x()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  shooter.ejectNote();
                  shooter.setFlywheelVelocity(-defaultFlywheelSpeedRPM); // Adjust RPM if needed
                },
                () -> {
                  shooter.stopFeeder();
                  shooter.stop();
                },
                shooter));

    // Right Bumper — INTAKE IN
    operator.rightBumper().whileTrue(intake.intakeCommand());

    // Left Bumper — INTAKE OUT
    operator.leftBumper().whileTrue(intake.outtakeCommand());

    // B — INDEXER IN
    operator.b().whileTrue(indexer.feedCommand());

    // Operator A — full auto-shoot (visual TX aim + spin up + feed)
    operator.a().whileTrue(new AutoShootCommand(drive, shooter, indexer, vision, 0));

    // POV Down — TOGGLE Slapdown (Down if Up, Up if Down) and rollers run (done inside
    // toggleSlapdown command)
    operator.povDown().onTrue(Commands.runOnce(intake::toggleSlapdown, intake));

    // POV Up — JITTER while held (Agitate balls) and run the intake rollers (done inside
    // jitterCommand)
    operator.povUp().whileTrue(intake.jitterCommand());

    // POV Left — manually nudge hood DOWN by 5 degree
    operator
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  double newAngle = shooter.getHoodPosition() - Units.degreesToRadians(5.0);
                  shooter.setHoodPosition(newAngle);

                  // debug prints
                  System.out.println(
                      "Hood calculated angle: " + Units.radiansToDegrees(newAngle) + " deg");
                  System.out.println(
                      "Hood actual angle: "
                          + Units.radiansToDegrees(shooter.getHoodPosition())
                          + " deg");
                },
                shooter));

    // POV Right — manually nudge hood UP by 1 degree (edited to 3 degrees)
    operator
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  double newAngle = shooter.getHoodPosition() + Units.degreesToRadians(5.0);
                  shooter.setHoodPosition(newAngle);

                  // debug prints
                  System.out.println(
                      "Hood calculated angle: " + Units.radiansToDegrees(newAngle) + " deg");
                  System.out.println(
                      "Hood actual angle: "
                          + Units.radiansToDegrees(shooter.getHoodPosition())
                          + " deg");
                },
                shooter));

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

    // Right Stick Press — Hardcoded Trench Shot Preset
    operator
        .rightStick()
        .whileTrue(
            Commands.run(
                    () -> {
                      // ADJUST THESE BASED ON TESTING PLS
                      double trenchHoodAngleRad = Units.degreesToRadians(25.0);
                      double trenchFlywheelRPM = 4500.0;

                      shooter.setHoodPosition(trenchHoodAngleRad);
                      shooter.setFlywheelVelocity(trenchFlywheelRPM);

                      // Only feed once flywheels and hood are ready
                      if (shooter.areFlywheelsAtSpeed() && shooter.isHoodAtPosition()) {
                        indexer.feed();
                        shooter.feedNote();
                      } else {
                        shooter.stopFeeder();
                        indexer.stop();
                      }
                    },
                    shooter,
                    indexer)
                .finallyDo(
                    () -> {
                      shooter.stop();
                      shooter.stopFeeder();
                      indexer.stop();
                    }));
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
