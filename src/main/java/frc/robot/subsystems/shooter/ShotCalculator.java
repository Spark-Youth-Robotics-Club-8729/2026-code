// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShotCalculator.ShootingParameters;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {
  private static ShotCalculator instance;

  // Interpolation maps
  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final double minDistance = 0.9; // meters
  private static final double maxDistance = 4.9; // meters

  // --- TUNING CONSTANTS ---
  // Flywheel: RPM = (Slope * distance) + Intercept
  private static final double FLYWHEEL_SLOPE = 285.0;
  private static final double FLYWHEEL_INTERCEPT = 1750.0;

  // Hood Angle (Quadratic): Radians = (A * d^2) + (B * d) + C
  private static final double HOOD_A = 0.021;
  private static final double HOOD_B = 0.015;
  private static final double HOOD_C = 0.145; // Base angle at close range

  // Time of Flight (Quadratic): Seconds = (A * d^2) + (B * d) + C
  private static final double TOF_A = -0.012;
  private static final double TOF_B = 0.142;
  private static final double TOF_C = 0.725;

  static {
    // Hood angle map — distance (meters) -> hood angle (radians)
    hoodAngleMap.put(0.96, Units.degreesToRadians(10.0));
    hoodAngleMap.put(1.16, Units.degreesToRadians(12.0));
    hoodAngleMap.put(1.58, Units.degreesToRadians(14.0));
    hoodAngleMap.put(2.07, Units.degreesToRadians(18.5));
    hoodAngleMap.put(2.37, Units.degreesToRadians(22.0));
    hoodAngleMap.put(2.47, Units.degreesToRadians(23.0));
    hoodAngleMap.put(2.70, Units.degreesToRadians(24.0));
    hoodAngleMap.put(2.94, Units.degreesToRadians(25.0));
    hoodAngleMap.put(3.48, Units.degreesToRadians(27.0));
    hoodAngleMap.put(3.92, Units.degreesToRadians(32.0));
    hoodAngleMap.put(4.35, Units.degreesToRadians(34.0));
    hoodAngleMap.put(4.84, Units.degreesToRadians(38.0));

    // Flywheel speed map — distance (meters) -> RPM
    // Note: reference uses ~150-190 in different units; scaled to RPM range for our Krakens
    flywheelSpeedMap.put(0.96, 2000.0);
    flywheelSpeedMap.put(1.16, 2100.0);
    flywheelSpeedMap.put(1.58, 2200.0);
    flywheelSpeedMap.put(2.07, 2350.0);
    flywheelSpeedMap.put(2.37, 2500.0);
    flywheelSpeedMap.put(2.47, 2500.0);
    flywheelSpeedMap.put(2.70, 2500.0);
    flywheelSpeedMap.put(2.94, 2650.0);
    flywheelSpeedMap.put(3.48, 2650.0);
    flywheelSpeedMap.put(3.92, 2800.0);
    flywheelSpeedMap.put(4.35, 2950.0);
    flywheelSpeedMap.put(4.84, 3100.0);

    // Time of flight map — distance (meters) -> seconds
    timeOfFlightMap.put(1.38, 0.90);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(5.68, 1.16);
  }

  /** Immutable result from a single calculate() call. */
  public record ShootingParameters(
      boolean isValid,
      double hoodAngleRad,
      double flywheelSpeedRPM,
      double distanceToTarget,
      double timeOfFlight,
      Rotation2d driveAngle) {}

  // Cached result — cleared each loop via clearParameters()
  private ShootingParameters latestParameters = null;

  // Hood angle trim offset (degrees) — can be incremented live for tuning
  private double hoodAngleOffsetDeg = 0.0;

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> velocitySupplier;
  private final Translation2d targetPosition;

  private ShotCalculator(
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> velocitySupplier,
      Translation2d targetPosition) {
    this.poseSupplier = poseSupplier;
    this.velocitySupplier = velocitySupplier;
    this.targetPosition = targetPosition;
  }

  /**
   * Initializes the singleton.
   *
   * @param poseSupplier Supplier for the current robot field pose.
   * @param velocitySupplier Supplier for the current robot-relative chassis speeds (used for
   *     lookahead). Pass {@code () -> new ChassisSpeeds()} if you don't want lookahead.
   * @param targetPosition Field position of the shoot target (e.g., speaker opening center).
   */
  public static void initialize(
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> velocitySupplier,
      Translation2d targetPosition) {
    if (instance == null) {
      instance = new ShotCalculator(poseSupplier, velocitySupplier, targetPosition);
    }
  }

  /** Returns the singleton. Throws if {@link #initialize} has not been called. */
  public static ShotCalculator getInstance() {
    if (instance == null) {
      throw new IllegalStateException(
          "ShotCalculator has not been initialized. Call ShotCalculator.initialize() first.");
    }
    return instance;
  }

  /**
   * Calculates (or returns cached) shooting parameters for this loop cycle. Call {@link
   * #clearParameters()} at the start of each loop to invalidate the cache.
   */
  public ShootingParameters calculate() {
    if (latestParameters != null) return latestParameters;

    Pose2d pose = poseSupplier.get();
    ChassisSpeeds velocity = velocitySupplier.get();

    // Convert robot-relative velocity to field-relative
    double fieldVx =
        velocity.vxMetersPerSecond * pose.getRotation().getCos()
            - velocity.vyMetersPerSecond * pose.getRotation().getSin();
    double fieldVy =
        velocity.vxMetersPerSecond * pose.getRotation().getSin()
            + velocity.vyMetersPerSecond * pose.getRotation().getCos();

    // Iterative lookahead: converge on where the note will land accounting for robot motion
    double lookaheadDistance = pose.getTranslation().getDistance(targetPosition);
    Translation2d lookaheadTranslation = pose.getTranslation();
    for (int i = 0;
        i < 5;
        i++) { // changed from 10 to 5, but if it doesnt work, then change back to 10
      double tof = timeOfFlightMap.get(lookaheadDistance);
      lookaheadTranslation =
          new Translation2d(
              pose.getTranslation().getX() + fieldVx * tof,
              pose.getTranslation().getY() + fieldVy * tof);
      lookaheadDistance = lookaheadTranslation.getDistance(targetPosition);
    }

    double finalDistance = lookaheadDistance;
    // double tof = timeOfFlightMap.get(finalDistance);
    double tof = calculateTOF(finalDistance); // using the equation now instead of presets

    // Drive angle: aim robot center at target from the lookahead position
    Rotation2d driveAngle = targetPosition.minus(lookaheadTranslation).getAngle();

    boolean isValid = finalDistance >= minDistance && finalDistance <= maxDistance;
    // double hoodAngle = hoodAngleMap.get(finalDistance) +
    // Units.degreesToRadians(hoodAngleOffsetDeg);
    // double flywheelSpeed = flywheelSpeedMap.get(finalDistance);
    double hoodAngle =
        calculateHoodAngle(finalDistance)
            + Units.degreesToRadians(
                hoodAngleOffsetDeg); // using the equation now instead of presets
    double flywheelSpeed =
        calculateFlywheelSpeed(finalDistance); // using the equation now instead of presets

    latestParameters =
        new ShootingParameters(isValid, hoodAngle, flywheelSpeed, finalDistance, tof, driveAngle);

    Logger.recordOutput("ShotCalculator/DistanceToTarget", finalDistance);
    Logger.recordOutput(
        "ShotCalculator/DistanceNoLookahead", pose.getTranslation().getDistance(targetPosition));
    Logger.recordOutput("ShotCalculator/IsValid", isValid);
    Logger.recordOutput("ShotCalculator/HoodAngleRad", hoodAngle);
    Logger.recordOutput("ShotCalculator/FlywheelSpeedRPM", flywheelSpeed);
    Logger.recordOutput("ShotCalculator/TimeOfFlight", tof);
    Logger.recordOutput("ShotCalculator/DriveAngle", driveAngle);

    return latestParameters;
  }

  /** Call once per loop (e.g. in a periodic) to invalidate the cached result. */
  public void clearParameters() {
    latestParameters = null;
  }

  /** Nudges the hood angle trim up or down for live tuning. */
  public void incrementHoodAngleOffset(double deltaDegrees) {
    hoodAngleOffsetDeg += deltaDegrees;
    Logger.recordOutput("ShotCalculator/HoodAngleOffsetDeg", hoodAngleOffsetDeg);
  }

  public double getHoodAngleOffsetDeg() {
    return hoodAngleOffsetDeg;
  }

  /** Returns true if the robot is currently within the valid shooting range. */
  public boolean isInRange() {
    return calculate().isValid();
  }

  /** Returns the straight-line distance from the robot to the target in meters. */
  public double getDistanceToTarget() {
    return poseSupplier.get().getTranslation().getDistance(targetPosition);
  }

  /** Returns the target field position. */
  public Translation2d getTargetPosition() {
    return targetPosition;
  }

  // The equationss for flywheel speed, hood angle, and time of flight are based on empirical data
  // and tuning. They can be adjusted as needed to better fit the actual performance of the robot.
  private double calculateFlywheelSpeed(double distance) {
    return (FLYWHEEL_SLOPE * distance) + FLYWHEEL_INTERCEPT;
  }

  private double calculateHoodAngle(double distance) {
    // a*d^2 + b*d + c
    return (HOOD_A * Math.pow(distance, 2)) + (HOOD_B * distance) + HOOD_C;
  }

  private double calculateTOF(double distance) {
    return (TOF_A * Math.pow(distance, 2)) + (TOF_B * distance) + TOF_C;
  }

  /**
   * Calculates shooting parameters directly from a vision-measured distance. Bypasses the lookahead
   * and pose math — use when the LL4 can see hub tags and provides a reliable direct range.
   *
   * @param distanceMeters Direct distance to the hub (e.g. from avgTagDistance).
   * @param driveAngle Desired robot heading toward the hub.
   * @return ShootingParameters with isValid based on range limits.
   */
  public ShootingParameters calculateFromDistance(
      double distanceMeters, edu.wpi.first.math.geometry.Rotation2d driveAngle) {
    boolean isValid = distanceMeters >= minDistance && distanceMeters <= maxDistance;
    double hoodAngle =
        hoodAngleMap.get(distanceMeters) + Units.degreesToRadians(hoodAngleOffsetDeg);
    double flywheelSpeed = flywheelSpeedMap.get(distanceMeters);
    double tof = timeOfFlightMap.get(distanceMeters);

    var params =
        new ShootingParameters(isValid, hoodAngle, flywheelSpeed, distanceMeters, tof, driveAngle);

    Logger.recordOutput("ShotCalculator/VisionDistance/DistanceM", distanceMeters);
    Logger.recordOutput("ShotCalculator/VisionDistance/IsValid", isValid);
    Logger.recordOutput("ShotCalculator/VisionDistance/HoodAngleRad", hoodAngle);
    Logger.recordOutput("ShotCalculator/VisionDistance/FlywheelSpeedRPM", flywheelSpeed);

    return params;
  }

  /**
   * Returns the hub target position for the current alliance. Falls back to the configured
   * targetPosition if alliance is unknown.
   */
  public Translation2d getAllianceHubPosition() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == Alliance.Red
          ? FieldConstants.Hub.oppInnerCenterPoint.toTranslation2d()
          : FieldConstants.Hub.innerCenterPoint.toTranslation2d();
    }
    return targetPosition; // fallback
  }

  /**
   * Returns a {@link Pose2d} at {@code robotPosition} aimed directly at the hub. Used by {@link
   * frc.robot.AutoFieldConstants} to pre-compute launch poses for autonomous.
   *
   * @param robotPosition Where the robot will be positioned on the field.
   * @param forBlue {@code true} to aim at the blue hub, {@code false} for the red hub.
   * @return A Pose2d with the robot at {@code robotPosition} rotated to face the hub.
   */
  public static Pose2d getStationaryAimedPose(Translation2d robotPosition, boolean forBlue) {
    Translation2d hubCenter =
        forBlue
            ? FieldConstants.Hub.innerCenterPoint.toTranslation2d()
            : FieldConstants.Hub.oppInnerCenterPoint.toTranslation2d();
    Rotation2d aimAngle = hubCenter.minus(robotPosition).getAngle();
    return new Pose2d(robotPosition, aimAngle);
  }
}
