package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ShooterConstants;

/** Physics for shooter calculations */
public class ShooterCalculations {
  /**
   * Calcualtes the requried initial velocity for the FUEL to be shot at so that it enters the HUB
   * in meters/second.
   *
   * @param deltaD The horizontal distance to the HUB, taken from the computer vision part of it
   * @param deltaH The vertical distance to the HUB (distance from shooter to center of HUB)
   * @param thetaDeg Shooting angle in degrees (optimal is 45 degrees).
   * @return The initial velocity, in meters/second
   */
  public static double calculateInitialVelocity(double deltaD, double deltaH, double thetaDeg) {
    double thetaRadians = Math.toRadians(thetaDeg);

    double numerator = deltaD * deltaD * ShooterConstants.GRAVITY;
    double denominator =
        2 * Math.pow(Math.cos(thetaRadians), 2) * (deltaD * Math.tan(thetaRadians) - deltaH);

    double initialVelocity = Math.sqrt(numerator / denominator);

    return initialVelocity; // This is in m/s
  }
  /**
   * Converts velocity in meters/second to RPM
   *
   * @param velocityMPS Velocity in meters/second, calculated from calcualteInitialVelocity
   * @return RPM at which the motor must spin in order to shoot the FUEL at that velocity
   */
  public static double velocityToRPM(double velocityMPS) {
    double wheelCircumference = Math.PI * ShooterConstants.WHEEL_DIAMETER;
    double rotationsPerSecond = velocityMPS / wheelCircumference;
    double rpm = rotationsPerSecond * 60.0;

    return rpm;
  }

  /**
   * Pulls the two above functions together and handles exceptions, as well as limits to the motor's
   * minimum and maximum RPM.
   *
   * @param distanceMeters Horizontal distance to the HUB
   * @return RPM clamped to the motor's max and min values
   */
  public static double calculateRPM(double distanceMeters) {
    double thetaDeg = ShooterConstants.LAUNCH_ANGLE;
    double deltaH = ShooterConstants.HEIGHT_DIFF;

    double velocity = calculateInitialVelocity(distanceMeters, deltaH, thetaDeg);

    if (velocity < 0) {
      return ShooterConstants.MIN_SHOOTER_RPM;
    }

    double rpm = velocityToRPM(velocity);

    return MathUtil.clamp(rpm, ShooterConstants.MIN_SHOOTER_RPM, ShooterConstants.MAX_SHOOTER_RPM);
  }
}
