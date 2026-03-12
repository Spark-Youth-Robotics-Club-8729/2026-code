package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;

/**
 * Factory class for full autonomous fuel-cycle routines.
 *
 * <p>Field layout (Blue alliance, WPILib coordinates — low-X = Blue side):
 *
 * <pre>
 *  X≈0       Alliance wall
 *  X≈1.1     Tower (climbing structure)
 *  X≈3.3     H — hub scoring position: robot faces +X and shoots fuel into
 *             the hub's near face (AprilTag 26). The hub opens toward the
 *             alliance wall side, so the robot stands BETWEEN the wall and
 *             the hub and shoots inward.
 *  X≈4.3     Hub near face / Blue starting line — S1–S4 start here, right
 *             at the hub near face.
 *  X≈5.5     Hub far face (field side)
 *  X≈5.7+    Neutral zone begins
 *  X≈6.8–8.0 N1–N6 — midfield fuel balls to collect
 *  X≈8.8     Field center
 * </pre>
 *
 * <p>Strategy: start at S, shoot preloaded fuel, drive out to midfield to intake loose fuel (N
 * positions), return to H (hub scoring spot), and shoot. Repeat for multi-cycle autos.
 *
 * <p>Path naming:
 *
 * <ul>
 *   <li>S1–S4 = starting positions, top → bottom (high-Y → low-Y, Blue side)
 *   <li>N1–N6 = neutral-zone fuel balls (N1/2 upper, N3/4 center, N5/6 lower)
 *   <li>H = hub scoring area (X≈3.3 — shoot fuel into the hub from here)
 *   <li>T1, T2 = top/upper routing waypoints (high-Y, route above the hub)
 *   <li>B1, B2 = bottom/lower routing waypoints (low-Y, route below the hub)
 * </ul>
 */
public final class AutoRoutines {

  // Max time a single shoot action is allowed before the sequence moves on
  private static final double SHOOT_TIMEOUT_SECONDS = 3.0;

  private AutoRoutines() {}

  // ---------------------------------------------------------------------------
  // Private helpers
  // ---------------------------------------------------------------------------

  /**
   * Loads and follows a PathPlanner path by name. Returns {@code Commands.none()} if the path file
   * cannot be loaded (logs a warning so the auto can still run partially).
   */
  private static Command followPath(String pathName) {
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
    } catch (Exception e) {
      System.err.println("[AutoRoutines] Failed to load path: " + pathName);
      return Commands.none();
    }
  }

  /**
   * Aims at the hub using Limelight TX, spins up the shooter via ShotCalculator, and fires one fuel
   * ball, then finishes. Hard timeout ensures the sequence always continues.
   */
  private static Command shootFuelIntoHub(
      Drive drive, Shooter shooter, Indexer indexer, Vision vision) {
    return new AutoShootCommand(drive, shooter, indexer, vision, 0)
        .withTimeout(SHOOT_TIMEOUT_SECONDS);
  }

  /**
   * Runs continuously: deploys the slapdown arm and spins the intake rollers to collect fuel.
   * Designed to run inside {@code Commands.deadline(...)} alongside a path so intake is active for
   * the entire drive to the midfield fuel position. Rollers stay on after this ends — call {@code
   * intake.retractCommand()} to stop them before shooting.
   */
  private static Command collectFuel(Intake intake) {
    return Commands.startEnd(
        () -> {
          intake.setSlapdownGoal(Intake.SlapdownGoal.DOWN);
          intake.setRollerGoal(Intake.RollerGoal.INTAKE);
        },
        () -> {}, // rollers stay running; retractCommand() cleans up
        intake);
  }

  /**
   * Runs continuously: pre-warms the flywheels to default speed while the robot drives back to H.
   * Designed to run inside {@code Commands.deadline(...)} so it stops when the path finishes.
   * {@link AutoShootCommand} takes over precise flywheel control for the actual shot.
   */
  private static Command preWarmShooter(Shooter shooter) {
    return Commands.run(
        () -> shooter.setFlywheelVelocity(ShooterConstants.defaultFlywheelSpeedRPM), shooter);
  }

  // ---------------------------------------------------------------------------
  // Public routine factories
  // ---------------------------------------------------------------------------

  /**
   * Shoot-only auto: fires the preloaded fuel ball from the starting position and stays put. Works
   * from any Blue starting position (S1–S4).
   */
  public static Command shootOnlyAuto(
      Drive drive, Shooter shooter, Indexer indexer, Vision vision) {
    return shootFuelIntoHub(drive, shooter, indexer, vision).withName("Shoot Only");
  }

  /**
   * 1-cycle fuel auto from S1 (top Blue start, Y≈7.4):
   *
   * <ol>
   *   <li>Shoot preloaded fuel from S1 into hub
   *   <li>Drive out to N1 (upper midfield fuel, X≈7.8) while deploying intake
   *   <li>Retract intake; drive back to hub scoring spot H (through T1) while shooter pre-warms
   *   <li>Shoot collected fuel into hub at H
   * </ol>
   */
  public static Command oneCycleFuelAutoS1(
      Drive drive, Shooter shooter, Indexer indexer, Intake intake, Vision vision) {
    return Commands.sequence(
            shootFuelIntoHub(drive, shooter, indexer, vision),
            Commands.deadline(followPath("BlueS1toN1"), collectFuel(intake)),
            intake.retractCommand(),
            Commands.deadline(followPath("BlueN1toHthroughT1"), preWarmShooter(shooter)),
            shootFuelIntoHub(drive, shooter, indexer, vision))
        .withName("1-Cycle Fuel Auto S1");
  }

  /**
   * 1-cycle fuel auto from S2 (center-upper Blue start, Y≈5.8):
   *
   * <ol>
   *   <li>Shoot preloaded fuel from S2 into hub
   *   <li>Drive out to N1 (upper midfield fuel, X≈7.8) while deploying intake
   *   <li>Retract intake; drive back to H (through T1) while shooter pre-warms
   *   <li>Shoot collected fuel into hub at H
   * </ol>
   */
  public static Command oneCycleFuelAutoS2(
      Drive drive, Shooter shooter, Indexer indexer, Intake intake, Vision vision) {
    return Commands.sequence(
            shootFuelIntoHub(drive, shooter, indexer, vision),
            Commands.deadline(followPath("BlueS2toN1"), collectFuel(intake)),
            intake.retractCommand(),
            Commands.deadline(followPath("BlueN1toHthroughT1"), preWarmShooter(shooter)),
            shootFuelIntoHub(drive, shooter, indexer, vision))
        .withName("1-Cycle Fuel Auto S2");
  }

  /**
   * 2-cycle fuel auto from S2 — top routing (routes above hub through T1):
   *
   * <ol>
   *   <li>Shoot preloaded fuel from S2
   *   <li>Collect N1 from upper midfield; return to H via T1 and shoot
   *   <li>Collect N2 from upper midfield; return to H via T1 and shoot
   * </ol>
   */
  public static Command twoCycleFuelAutoS2Top(
      Drive drive, Shooter shooter, Indexer indexer, Intake intake, Vision vision) {
    return Commands.sequence(
            // Shoot preload
            shootFuelIntoHub(drive, shooter, indexer, vision),

            // Fuel cycle 1: N1
            Commands.deadline(followPath("BlueS2toN1"), collectFuel(intake)),
            intake.retractCommand(),
            Commands.deadline(followPath("BlueN1toHthroughT1"), preWarmShooter(shooter)),
            shootFuelIntoHub(drive, shooter, indexer, vision),

            // Fuel cycle 2: N2
            Commands.deadline(followPath("BlueHtoN2throughT1"), collectFuel(intake)),
            intake.retractCommand(),
            Commands.deadline(followPath("BlueN2toHthroughT1"), preWarmShooter(shooter)),
            shootFuelIntoHub(drive, shooter, indexer, vision))
        .withName("2-Cycle Fuel Auto S2 (Top)");
  }

  /**
   * 2-cycle fuel auto from S2 — bottom routing (routes below hub through B1):
   *
   * <ol>
   *   <li>Shoot preloaded fuel from S2
   *   <li>Collect N1; return to H via B1 and shoot
   *   <li>Collect N2 from H; return to H via B1 and shoot
   * </ol>
   */
  public static Command twoCycleFuelAutoS2Bottom(
      Drive drive, Shooter shooter, Indexer indexer, Intake intake, Vision vision) {
    return Commands.sequence(
            shootFuelIntoHub(drive, shooter, indexer, vision),
            Commands.deadline(followPath("BlueS2toN1"), collectFuel(intake)),
            intake.retractCommand(),
            Commands.deadline(followPath("BlueN1toHthroughB1"), preWarmShooter(shooter)),
            shootFuelIntoHub(drive, shooter, indexer, vision),
            Commands.deadline(followPath("BlueHtoN2throughB1"), collectFuel(intake)),
            intake.retractCommand(),
            Commands.deadline(followPath("BlueN2toHthroughB1"), preWarmShooter(shooter)),
            shootFuelIntoHub(drive, shooter, indexer, vision))
        .withName("2-Cycle Fuel Auto S2 (Bottom)");
  }

  /**
   * 1-cycle fuel auto from S3 (center-lower Blue start):
   *
   * <ol>
   *   <li>Shoot preloaded fuel from S3
   *   <li>Drive out to N3 (center midfield fuel) while deploying intake
   *   <li>Retract intake; drive back to H (through B2) while shooter pre-warms
   *   <li>Shoot collected fuel into hub at H
   * </ol>
   */
  public static Command oneCycleFuelAutoS3(
      Drive drive, Shooter shooter, Indexer indexer, Intake intake, Vision vision) {
    return Commands.sequence(
            shootFuelIntoHub(drive, shooter, indexer, vision),
            Commands.deadline(followPath("BlueS3toN3"), collectFuel(intake)),
            intake.retractCommand(),
            Commands.deadline(followPath("BlueN3toHthroughB2"), preWarmShooter(shooter)),
            shootFuelIntoHub(drive, shooter, indexer, vision))
        .withName("1-Cycle Fuel Auto S3");
  }

  /**
   * 2-cycle fuel auto from S3 — collects N3 then N4, both via B2 (lower routing):
   *
   * <ol>
   *   <li>Shoot preloaded fuel from S3
   *   <li>Collect N3 from midfield; return to H via B2 and shoot
   *   <li>Collect N4 from midfield; return to H via B2 and shoot
   * </ol>
   */
  public static Command twoCycleFuelAutoS3(
      Drive drive, Shooter shooter, Indexer indexer, Intake intake, Vision vision) {
    return Commands.sequence(
            shootFuelIntoHub(drive, shooter, indexer, vision),
            Commands.deadline(followPath("BlueS3toN3"), collectFuel(intake)),
            intake.retractCommand(),
            Commands.deadline(followPath("BlueN3toHthroughB2"), preWarmShooter(shooter)),
            shootFuelIntoHub(drive, shooter, indexer, vision),
            Commands.deadline(followPath("BlueHtoN4throughB2"), collectFuel(intake)),
            intake.retractCommand(),
            Commands.deadline(followPath("BlueN4toHthroughB2"), preWarmShooter(shooter)),
            shootFuelIntoHub(drive, shooter, indexer, vision))
        .withName("2-Cycle Fuel Auto S3");
  }

  /**
   * 1-cycle fuel auto from S4 (bottom-right Blue start, Y≈0.7):
   *
   * <ol>
   *   <li>Shoot preloaded fuel from S4
   *   <li>Drive out to N4 (lower midfield fuel) while deploying intake
   *   <li>Retract intake; drive back to H (through B2) while shooter pre-warms
   *   <li>Shoot collected fuel into hub at H
   * </ol>
   */
  public static Command oneCycleFuelAutoS4(
      Drive drive, Shooter shooter, Indexer indexer, Intake intake, Vision vision) {
    return Commands.sequence(
            shootFuelIntoHub(drive, shooter, indexer, vision),
            Commands.deadline(followPath("BlueS4toN4"), collectFuel(intake)),
            intake.retractCommand(),
            Commands.deadline(followPath("BlueN4toHthroughB2"), preWarmShooter(shooter)),
            shootFuelIntoHub(drive, shooter, indexer, vision))
        .withName("1-Cycle Fuel Auto S4");
  }
}
