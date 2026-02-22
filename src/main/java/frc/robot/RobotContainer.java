// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKrakenX60;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);

  // Aim controller for vision-based rotation
  private final PIDController aimController = new PIDController(0.2, 0.0, 0.0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // ---------------------------------------------------------------------------
  // TODO: Set this to the actual field position of your shoot target (e.g. the
  // center of the speaker opening on your field layout).
  // ---------------------------------------------------------------------------
  private static final Translation2d SHOOT_TARGET_POSITION = new Translation2d(0.0, 5.55);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));

        shooter = new Shooter(new ShooterIOKrakenX60());
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        shooter = new Shooter(new ShooterIOSim());
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
        break;
    }

    // Initialize the shot calculator singleton once we have a pose supplier
    ShotCalculator.initialize(drive::getPose, SHOOT_TARGET_POSITION);

    // Set up auto routines
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
    // ------------------------------------------------------------------
    // DRIVE — default field-relative joystick drive
    // ------------------------------------------------------------------
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // X-pattern brake when X is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Auto-aim to AprilTag using vision when Y button is held
    aimController.enableContinuousInput(-Math.PI, Math.PI);
    controller
        .y()
        .whileTrue(
            Commands.startRun(
                () -> aimController.reset(),
                () -> {
                  double rotationSpeed =
                      aimController.calculate(vision.getTargetX(0).getRadians());
                  drive.setDefaultCommand(
                      DriveCommands.joystickDrive(
                          drive,
                          () -> -controller.getLeftY(),
                          () -> -controller.getLeftX(),
                          () -> rotationSpeed));
                },
                drive));

    // Left bumper: Reset odometry using vision
    controller
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> System.out.println("Reset odometry using vision (not implemented)"), drive));

    // Right bumper: Toggle vision pipeline
    controller
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> System.out.println("Toggle vision pipeline (not implemented)")));

    // ------------------------------------------------------------------
    // SHOOTER
    // ------------------------------------------------------------------

    // Right trigger: spin up flywheel + aim hood at current distance,
    // then feed the ball when ready.
    // Hold to keep spinning; release to stop everything.
    controller
        .rightTrigger(0.5)
        .whileTrue(
            Commands.run(
                    () -> {
                      // Recalculate each loop so hood/flywheel track as the robot moves
                      var params = ShotCalculator.getInstance().calculate();
                      shooter.setHoodPosition(params.hoodAngleRad());
                      shooter.setFlywheelVelocity(params.flywheelSpeedRPM());

                      // Only feed once everything is settled and in range
                      if (params.isValid() && shooter.isReadyToShoot()) {
                        shooter.feedNote();
                      } else {
                        shooter.stopIndexer();
                      }
                    },
                    shooter)
                .finallyDo(shooter::stop));

    // Left trigger: eject ball back toward hopper
    controller
        .leftTrigger(0.5)
        .whileTrue(
            Commands.startEnd(shooter::ejectNote, shooter::stopIndexer, shooter));

    // Right bumper (secondary use on operator controller if you add one):
    // For now, left stick button manually spins up to default speed without
    // hood tracking — useful for testing the flywheel independently.
    controller
        .leftStick()
        .whileTrue(
            Commands.startEnd(
                    () -> shooter.setFlywheelVelocity(
                        frc.robot.subsystems.shooter.ShooterConstants.defaultFlywheelSpeedRPM),
                    shooter::stop,
                    shooter));
  }

  /** Use this to pass the autonomous command to the main {@link Robot} class. */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Returns the drive subsystem. */
  public Drive getDrive() {
    return drive;
  }

  /** Returns the vision subsystem. */
  public Vision getVision() {
    return vision;
  }

  /** Returns the shooter subsystem. */
  public Shooter getShooter() {
    return shooter;
  }
}