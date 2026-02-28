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

  private static final double minDistance = 1.0; // meters
  private static final double maxDistance = 6.0; // meters

  static {
    // TODO: Replace with values tuned on the actual robot.
    // Format: distance (meters) -> value
    hoodAngleMap.put(1.0, Units.degreesToRadians(15.0));
    hoodAngleMap.put(1.5, Units.degreesToRadians(18.0));
    hoodAngleMap.put(2.0, Units.degreesToRadians(22.0));
    hoodAngleMap.put(2.5, Units.degreesToRadians(26.0));
    hoodAngleMap.put(3.0, Units.degreesToRadians(30.0));
    hoodAngleMap.put(3.5, Units.degreesToRadians(34.0));
    hoodAngleMap.put(4.0, Units.degreesToRadians(38.0));
    hoodAngleMap.put(4.5, Units.degreesToRadians(42.0));
    hoodAngleMap.put(5.0, Units.degreesToRadians(46.0));
    hoodAngleMap.put(5.5, Units.degreesToRadians(50.0));
    hoodAngleMap.put(6.0, Units.degreesToRadians(54.0));

    flywheelSpeedMap.put(1.0, 2000.0);
    flywheelSpeedMap.put(1.5, 2200.0);
    flywheelSpeedMap.put(2.0, 2400.0);
    flywheelSpeedMap.put(2.5, 2600.0);
    flywheelSpeedMap.put(3.0, 2800.0);
    flywheelSpeedMap.put(3.5, 3000.0);
    flywheelSpeedMap.put(4.0, 3200.0);
    flywheelSpeedMap.put(4.5, 3400.0);
    flywheelSpeedMap.put(5.0, 3600.0);
    flywheelSpeedMap.put(5.5, 3800.0);
    flywheelSpeedMap.put(6.0, 4000.0);

    // Time of flight (seconds) vs distance (meters) — TODO: Tune on actual robot
    timeOfFlightMap.put(1.0, 0.3);
    timeOfFlightMap.put(2.0, 0.5);
    timeOfFlightMap.put(3.0, 0.7);
    timeOfFlightMap.put(4.0, 0.9);
    timeOfFlightMap.put(5.0, 1.1);
    timeOfFlightMap.put(6.0, 1.3);
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
    for (int i = 0; i < 10; i++) {
      double tof = timeOfFlightMap.get(lookaheadDistance);
      lookaheadTranslation =
          new Translation2d(
              pose.getTranslation().getX() + fieldVx * tof,
              pose.getTranslation().getY() + fieldVy * tof);
      lookaheadDistance = lookaheadTranslation.getDistance(targetPosition);
    }

    double finalDistance = lookaheadDistance;
    double tof = timeOfFlightMap.get(finalDistance);

    // Drive angle: aim robot center at target from the lookahead position
    Rotation2d driveAngle = targetPosition.minus(lookaheadTranslation).getAngle();

    boolean isValid = finalDistance >= minDistance && finalDistance <= maxDistance;
    double hoodAngle = hoodAngleMap.get(finalDistance) + Units.degreesToRadians(hoodAngleOffsetDeg);
    double flywheelSpeed = flywheelSpeedMap.get(finalDistance);

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
          ? frc.robot.subsystems.vision.VisionConstants.RED_HUB_POSITION
          : frc.robot.subsystems.vision.VisionConstants.BLUE_HUB_POSITION;
    }
    return targetPosition; // fallback
  }
}
