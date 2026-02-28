package frc.robot.util;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.Vision;

/**
 * Estimates distance to a vision target using the fixed-angle camera formula:
 *
 * <pre>
 *   d = (targetHeightIn - cameraHeightIn) / tan(mountAngleDeg + tyDeg)
 * </pre>
 *
 * <p>All height values are in inches for ease of physical measurement;
 * the returned distance is in meters.
 *
 * <p>This is only meaningful when the camera mount height differs noticeably
 * from the target height. If heights are similar, use target area instead.
 */
public class LimelightDistanceEstimator {

  private final Vision vision;
  private final int cameraIndex;

  /** Height of the Limelight lens above the floor (inches). TODO: measure on real robot */
  private final double cameraHeightInches;

  /** Height of the vision target above the floor (inches). TODO: set to actual target height */
  private final double targetHeightInches;

  /**
   * Angle of the camera above perfectly horizontal (degrees, positive = tilted up).
   * TODO: measure on real robot
   */
  private final double mountAngleDegrees;

  public LimelightDistanceEstimator(
      Vision vision,
      int cameraIndex,
      double cameraHeightInches,
      double targetHeightInches,
      double mountAngleDegrees) {
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    this.cameraHeightInches = cameraHeightInches;
    this.targetHeightInches = targetHeightInches;
    this.mountAngleDegrees = mountAngleDegrees;
  }

  /**
   * Returns estimated distance from the camera to the target in meters.
   * Returns {@link Double#NaN} if no target is visible or the geometry is degenerate.
   */
  public double getDistanceMeters() {
    if (!vision.hasTarget(cameraIndex)) {
      return Double.NaN;
    }

    double tyDeg = vision.getTargetY(cameraIndex).getDegrees();
    double angleRad = Units.degreesToRadians(mountAngleDegrees + tyDeg);

    double tanAngle = Math.tan(angleRad);
    if (Math.abs(tanAngle) < 1e-6) {
      return Double.NaN; // avoid division by zero
    }

    double distanceInches = (targetHeightInches - cameraHeightInches) / tanAngle;
    return Units.inchesToMeters(distanceInches);
  }

  /**
   * Returns estimated distance in inches, or {@link Double#NaN} if unavailable.
   */
  public double getDistanceInches() {
    double meters = getDistanceMeters();
    return Double.isNaN(meters) ? Double.NaN : Units.metersToInches(meters);
  }
}
