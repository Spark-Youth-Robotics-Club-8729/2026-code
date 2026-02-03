// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final
  class ShooterConstants { // TODO: Replace all below values with correct values and fine tune them
    // CAN IDs
    public static final int SHOOTER_MOTOR_ID = 0;
    public static final int SHOOTER_ANGLE_MOTOR_ID = 0;

    // PID Constants
    public static final double SHOOTER_KP = 0;
    public static final double SHOOTER_KI = 0;
    public static final double SHOOTER_KD = 0;
    public static final double SHOOTER_KF = 0;

    // Physical Limits
    public static final double MIN_SHOOTER_RPM = 0;
    public static final double MAX_SHOOTER_RPM = 0;
    public static final double SHOOTER_TOLERANCE_RPM = 0;
    public static final double WHEEL_DIAMETER =
        0; // In meters TODO: Change to actual wheel diameter from mech

    public static final double LAUNCH_ANGLE = 45; // In degrees
    public static final double GRAVITY = 9.8067; // In m/s^2
    public static final double SHOOTER_HEIGHT =
        0.75; // In meters TODO: Change to actual shooter height from mech
    public static final double HUB_CENTER_HEIGHT = 2.36; // Center of HUB, in meters
    public static final double HEIGHT_DIFF = HUB_CENTER_HEIGHT - SHOOTER_HEIGHT; // In meters
    public static final double FLIGHT_TIME = 0.5; // In seconds, experimentally determined
    // RMP depentent on distance will be calcualted with the formula in
    // ./src/main/java/frc/robot/util/ShooterCalculations.java
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
