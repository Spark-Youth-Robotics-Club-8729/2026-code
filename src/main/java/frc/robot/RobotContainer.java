// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
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
                // vision.getTargetX(0) returns the tx offset; negate to turn toward the tag
                () -> drive.getRotation().plus(vision.getTargetX(0))));

    // -------------------------------------------------------------------------
    // OPERATOR (port 1)
    // -------------------------------------------------------------------------

    // Right trigger — spin up + aim hood, fire when ready
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
                        shooter.stopIndexer();
                      }
                    },
                    shooter)
                .finallyDo(shooter::stop));

    // Left trigger — eject ball back toward hopper
    operator
        .leftTrigger(0.5)
        .whileTrue(Commands.startEnd(shooter::ejectNote, shooter::stopIndexer, shooter));

    // Right bumper — deploy slapdown + run roller; retract when released
    operator
        .rightBumper()
        .whileTrue(intake.slapdownAndIntakeCommand())
        .onFalse(intake.retractCommand());

    // Left bumper — roller only (arm already down), stops on release
    operator.leftBumper().whileTrue(intake.intakeCommand());

    // A — outtake
    operator.a().whileTrue(intake.outtakeCommand());

    // Left stick button — test flywheel at default speed
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
}
