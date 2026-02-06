// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Physics for shooter calculations with both static and moving shots. */
public class ShooterCalculations {

  /** Result of a moving shot calculation. */
  public static class MovingShotResult {
    /** Robot heading angle to face the hub (degrees) */
    public final double robotHeadingDeg;
    /** Shooter horizontal angle on x-y plane (2D plane) (degrees) */
    public final double shooterAngleDeg;
    /** Shooter elevation angle from horizontal (degrees, 0 to 90) */
    public final double elevationAngleDeg;
    /** Launch velocity in m/s */
    public final double launchVelocity;
    /** Required RPM clamped to motor limits */
    public final double rpm;

    public MovingShotResult(
        double robotHeadingDeg,
        double shooterAngleDeg,
        double elevationAngleDeg,
        double launchVelocity,
        double rpm) {
      this.robotHeadingDeg = robotHeadingDeg;
      this.shooterAngleDeg = shooterAngleDeg;
      this.elevationAngleDeg = elevationAngleDeg;
      this.launchVelocity = launchVelocity;
      this.rpm = rpm;
    }

    /** @return Robot heading in radians */
    public double getRobotHeadingRad() {
      return Math.toRadians(robotHeadingDeg);
    }

    /** @return Shooter angle in radians */
    public double getShooterAngleRad() {
      return Math.toRadians(shooterAngleDeg);
    }

    /** @return Elevation angle in radians */
    public double getElevationAngleRad() {
      return Math.toRadians(elevationAngleDeg);
    }
  }

  /**
   * Calculates shooter parameters for a shot while the robot is moving.
   *
   * <p>Uses projectile motion with velocity vector addition to account for robot movement.
   *
   * <p>The projectile velocity in the field frame equals the vector sum of the robot's velocity and
   * the launch velocity:
   * - v_x = v_rx + v0*cos(alpha)*cos(theta_shooter)
   * - v_y = v_ry + v0*cos(alpha)*sin(theta_shooter)
   * - v_z = v0*sin(alpha)
   *
   * @param robotPose Current robot pose (x, y, heading)
   * @param chassisSpeeds Chassis speeds relative to the floor (v_rx, v_ry, omega)
   * @param hubPosition Hub position on the field
   * @return MovingShotResult containing all required angles and velocities
   */
  public static MovingShotResult calculateMovingShot(
      Pose2d robotPose, ChassisSpeeds chassisSpeeds, Translation2d hubPosition) {

    // Calculate deltas
    double deltaX = hubPosition.getX() - robotPose.getX();
    double deltaY = hubPosition.getY() - robotPose.getY();
    double deltaD = Math.hypot(deltaX, deltaY); // horizontal distance
    double deltaH = ShooterConstants.heightDiffMeters; // vertical height difference

    // Robot velocity components
    double vRx = chassisSpeeds.vxMetersPerSecond;
    double vRy = chassisSpeeds.vyMetersPerSecond;

    // Robot heading to face the hub
    double robotHeading = Math.atan2(deltaY, deltaX);

    // Direction to hub
    double thetaHub = Math.atan2(deltaY, deltaX);

    // Required horizontal velocity magnitude (using flight time)
    double T = ShooterConstants.flightTimeSeconds;
    double vH = deltaD / T;

    // Horizontal component of launch velocity (in field frame, pointing to hub)
    double vHubX = vH * Math.cos(thetaHub);
    double vHubY = vH * Math.sin(thetaHub);

    // Horizontal launch velocity relative to robot (vector subtraction)
    double vLaunchX = vHubX - vRx;
    double vLaunchY = vHubY - vRy;

    // Shooter horizontal angle (direction to aim on x-y plane)
    double shooterAngle = Math.atan2(vLaunchY, vLaunchX);

    // Horizontal component magnitude of launch velocity
    double v0CosAlpha = Math.hypot(vLaunchX, vLaunchY);

    // Vertical component of launch velocity
    // from: deltaH = v0*sin(alpha)*T - 0.5*g*T^2
    // solving: v0*sin(alpha) = (deltaH + 0.5*g*T^2) / T = deltaH/T + g*T/2
    double v0SinAlpha = deltaH / T + ShooterConstants.gravity * T / 2.0;

    // Total launch velocity
    double v0 = Math.hypot(v0CosAlpha, v0SinAlpha);

    // Elevation angle
    double elevationAngle = Math.atan2(v0SinAlpha, v0CosAlpha);

    // Convert to RPM
    double rpm = velocityToRPM(v0);
    rpm = MathUtil.clamp(rpm, ShooterConstants.minFlywheelRPM, ShooterConstants.maxFlywheelRPM);

    // Convert radians to degrees for the result
    return new MovingShotResult(
        Math.toDegrees(robotHeading),
        Math.toDegrees(shooterAngle),
        Math.toDegrees(elevationAngle),
        v0,
        rpm);
  }

  /**
   * Calculates the required initial velocity for a STATIC shot (robot not moving).
   *
   * @param deltaD The horizontal distance to the HUB
   * @param deltaH The vertical distance to the HUB (shooter to center of HUB)
   * @param thetaDeg Shooting angle in degrees (optimal is 45 degrees)
   * @return The initial velocity in m/s
   */
  public static double calculateStaticInitialVelocity(
      double deltaD, double deltaH, double thetaDeg) {
    double thetaRadians = Math.toRadians(thetaDeg);

    double numerator = deltaD * deltaD * ShooterConstants.gravity;
    double denominator =
        2 * Math.pow(Math.cos(thetaRadians), 2) * (deltaD * Math.tan(thetaRadians) - deltaH);

    double initialVelocity = Math.sqrt(numerator / denominator);

    return initialVelocity;
  }

  /**
   * Converts velocity in m/s to RPM.
   *
   * @param velocityMPS Velocity in m/s
   * @return RPM at which the motor must spin
   */
  public static double velocityToRPM(double velocityMPS) {
    double wheelCircumference = Math.PI * ShooterConstants.wheelDiameterMeters;
    double rotationsPerSecond = velocityMPS / wheelCircumference;
    double rpm = rotationsPerSecond * 60.0;

    return rpm;
  }

  /**
   * Calculates RPM for a stationary shot at the given distance.
   *
   * @param distanceMeters Horizontal distance to the HUB
   * @return RPM clamped to motor limits
   */
  public static double calculateStaticRPM(double distanceMeters) {
    double thetaDeg = ShooterConstants.defaultLaunchAngleDeg;
    double deltaH = ShooterConstants.heightDiffMeters;

    double velocity = calculateStaticInitialVelocity(distanceMeters, deltaH, thetaDeg);

    if (Double.isNaN(velocity) || velocity < 0) {
      return ShooterConstants.minFlywheelRPM;
    }

    double rpm = velocityToRPM(velocity);

    return MathUtil.clamp(rpm, ShooterConstants.minFlywheelRPM, ShooterConstants.maxFlywheelRPM);
  }

  /**
   * Calculates the horizontal distance from robot to hub.
   *
   * @param robotPose Current robot pose
   * @param hubPosition Hub position on the field
   * @return Horizontal distance in meters
   */
  public static double calculateDistance(Pose2d robotPose, Translation2d hubPosition) {
    return robotPose.getTranslation().getDistance(hubPosition);
  }

  /**
   * Calculates the angle the robot should face to point at the hub.
   *
   * @param robotPose Current robot pose
   * @param hubPosition Hub position on the field
   * @return Angle in degrees
   */
  public static double calculateAngleToHub(Pose2d robotPose, Translation2d hubPosition) {
    double deltaX = hubPosition.getX() - robotPose.getX();
    double deltaY = hubPosition.getY() - robotPose.getY();
    return Math.toDegrees(Math.atan2(deltaY, deltaX));
  }

  /**
   * Calculates the angle the robot should face to point at the hub.
   *
   * @param robotPose Current robot pose
   * @param hubPosition Hub position on the field
   * @return Angle in radians
   */
  public static double calculateAngleToHubRad(Pose2d robotPose, Translation2d hubPosition) {
    double deltaX = hubPosition.getX() - robotPose.getX();
    double deltaY = hubPosition.getY() - robotPose.getY();
    return Math.atan2(deltaY, deltaX);
  }
}
