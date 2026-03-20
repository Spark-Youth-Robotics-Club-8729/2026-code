package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  // PID: kP — how aggressively to correct; kI/kD start at 0, tune if needed
  private static final double KP = 0.4;
  private static final double KI = 0.0;
  private static final double KD = 0.05;

  // Maximum rotation speed the PID output is clamped to (rad/s)
  private static final double MAX_OMEGA_RAD_S = 0.4;

  // Tolerance: consider aligned when TX is within ±1 degree
  private static final double TOLERANCE_DEG = 1.0;

  private final Drive drive;
  private final Vision vision;
  private final int cameraIndex;
  private final PIDController pid;

  public LimelightAimAndRangeCommand(Drive drive, Vision vision, int cameraIndex) {
    this.drive = drive;
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    this.pid = new PIDController(KP, KI, KD);
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
