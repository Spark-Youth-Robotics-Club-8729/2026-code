// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Calculates shooting parameters (hood angle, flywheel speed) from distance to target using
 * interpolation tables.
 *
 * <p>Usage: call {@code ShotCalculator.initialize(poseSupplier, targetPosition)} once at startup,
 * then {@code ShotCalculator.getInstance().calculate()} each loop.
 */
public class ShotCalculator {
  private static ShotCalculator instance;

  // Interpolation maps
  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
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
  }

  /** Immutable result from a single calculate() call. */
  public record ShootingParameters(
      boolean isValid, double hoodAngleRad, double flywheelSpeedRPM, double distanceToTarget) {}

  private final Supplier<Pose2d> poseSupplier;
  private final Translation2d targetPosition;

  private ShotCalculator(Supplier<Pose2d> poseSupplier, Translation2d targetPosition) {
    this.poseSupplier = poseSupplier;
    this.targetPosition = targetPosition;
  }

  /**
   * Initializes the singleton. Must be called before {@link #getInstance()}.
   *
   * @param poseSupplier Supplier that returns the current robot pose.
   * @param targetPosition Field position of the shoot target (e.g., speaker center).
   */
  public static void initialize(Supplier<Pose2d> poseSupplier, Translation2d targetPosition) {
    if (instance == null) {
      instance = new ShotCalculator(poseSupplier, targetPosition);
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
   * Calculates shooting parameters for the current robot pose.
   *
   * @return ShootingParameters with hood angle (rad), flywheel speed (RPM), and a validity flag.
   */
  public ShootingParameters calculate() {
    Pose2d currentPose = poseSupplier.get();
    double distance = currentPose.getTranslation().getDistance(targetPosition);

    boolean isValid = distance >= minDistance && distance <= maxDistance;
    double hoodAngle = hoodAngleMap.get(distance);
    double flywheelSpeed = flywheelSpeedMap.get(distance);

    Logger.recordOutput("ShotCalculator/DistanceToTarget", distance);
    Logger.recordOutput("ShotCalculator/IsValid", isValid);
    Logger.recordOutput("ShotCalculator/HoodAngleRad", hoodAngle);
    Logger.recordOutput("ShotCalculator/FlywheelSpeedRPM", flywheelSpeed);

    return new ShootingParameters(isValid, hoodAngle, flywheelSpeed, distance);
  }

  /** Returns true if the robot is currently within the valid shooting range. */
  public boolean isInRange() {
    double distance = poseSupplier.get().getTranslation().getDistance(targetPosition);
    return distance >= minDistance && distance <= maxDistance;
  }

  /** Returns the straight-line distance from the robot to the target in meters. */
  public double getDistanceToTarget() {
    return poseSupplier.get().getTranslation().getDistance(targetPosition);
  }

  /** Returns the target field position. */
  public Translation2d getTargetPosition() {
    return targetPosition;
  }
}
