package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * Rotates the robot to center the best Limelight target (TX → 0). No translation at all. Uses a PID
 * controller so it decelerates smoothly as it approaches center.
 *
 * <p>Bind via {@code .whileTrue()}.
 */
public class LimelightAimAndRangeCommand extends Command {

  // PID default values (can be tuned live via NetworkTables at
  // /LimelightAim/KP, /LimelightAim/KI, /LimelightAim/KD)
  private static final double DEFAULT_KP = 0.009;
  private static final double DEFAULT_KI = 0.0;
  private static final double DEFAULT_KD = 0.0;

  // Maximum rotation speed the PID output is clamped to (rad/s)
  private static final double MAX_OMEGA_RAD_S = 1.0; // was 0.4

  // Tolerance: consider aligned when TX is within ±2 degree
  private static final double TOLERANCE_DEG = 2.0;

  private final Drive drive;
  private final Vision vision;
  private final int cameraIndex;
  private final PIDController pid;
  private final NetworkTableEntry kpEntry;
  private final NetworkTableEntry kiEntry;
  private final NetworkTableEntry kdEntry;

  public LimelightAimAndRangeCommand(Drive drive, Vision vision, int cameraIndex) {
    this.drive = drive;
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    // NetworkTables entries for live PID tuning
    NetworkTable table = NetworkTableInstance.getDefault().getTable("LimelightAim");
    kpEntry = table.getEntry("KP");
    kiEntry = table.getEntry("KI");
    kdEntry = table.getEntry("KD");
    // Initialize entries with defaults (won't overwrite if already set by dashboard)
    kpEntry.setDouble(kpEntry.getDouble(DEFAULT_KP));
    kiEntry.setDouble(kiEntry.getDouble(DEFAULT_KI));
    kdEntry.setDouble(kdEntry.getDouble(DEFAULT_KD));

    this.pid = new PIDController(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    this.pid.setSetpoint(0.0); // target: TX = 0 (tag centered)
    this.pid.setTolerance(TOLERANCE_DEG);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    pid.reset();
  }

  @Override
  public void execute() {
    if (!vision.hasTarget(cameraIndex)) {
      // No target — stop and wait
      drive.stop();
      Logger.recordOutput("LimelightAimRange/HasTarget", false);
      return;
    }

    // Update PID gains from NetworkTables (allows live tuning without redeploy)
    double kp = kpEntry.getDouble(DEFAULT_KP);
    double ki = kiEntry.getDouble(DEFAULT_KI);
    double kd = kdEntry.getDouble(DEFAULT_KD);
    pid.setP(kp);
    pid.setI(ki);
    pid.setD(kd);

    double txDeg = vision.getTargetX(cameraIndex).getDegrees();

    // PID calculates correction: positive TX (tag right) → rotate CW (negative omega)
    double omega = MathUtil.clamp(pid.calculate(txDeg), -MAX_OMEGA_RAD_S, MAX_OMEGA_RAD_S);

    Logger.recordOutput("LimelightAimRange/HasTarget", true);
    Logger.recordOutput("LimelightAimRange/TX", txDeg);
    Logger.recordOutput("LimelightAimRange/OmegaRadS", omega);
    Logger.recordOutput("LimelightAimRange/AtSetpoint", pid.atSetpoint());

    // Rotation only — no translation
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
