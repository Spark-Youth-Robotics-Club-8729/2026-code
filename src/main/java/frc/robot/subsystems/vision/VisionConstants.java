// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Set;

public class VisionConstants {
  // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  // Limelight 4 camera name — must match the name set in the Limelight web UI
  public static String camera0Name = "limelight";

  // LL4 IMU modes:
  //   0 = External IMU only (default, we push robot orientation every loop)
  //   1 = Seed internal IMU from external (call once while disabled)
  //   4 = Internal IMU + external IMU assist (best accuracy during matches)
  public static final int imuModeDisabled = 1;   // seed on startup
  public static final int imuModeEnabled  = 4;   // internal + assist during match

  // How strongly the external gyro corrects the LL4 internal IMU (0.001 is gentle)
  public static final double imuAssistAlpha = 0.001;

  // MegaTag1: trust if 2+ tags visible
  public static final int megatag1MinTags = 2;

  // MegaTag2: trust even with 1 tag because gyro constrains the problem
  public static final int megatag2MinTags = 1;

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for camera
  public static double cameraStdDevFactor = 1.0;

  // MegaTag 2 multipliers — linear is better, angular is unavailable
  public static double linearStdDevMegatag2Factor = 0.5;
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;

  // Heartbeat timeout — if heartbeat hasn't changed in this many ms, camera is disconnected
  public static final long heartbeatTimeoutMs = 500;

  // -----------------------------------------------------------------------
  // Hub / Reef AprilTag ID sets (2026 field, see field image)
  // -----------------------------------------------------------------------
  // RED scoring hub face tags
  public static final Set<Integer> redHubTagIds = Set.of(5, 8, 9, 10, 3, 4, 2, 11);
  // BLUE scoring hub face tags
  public static final Set<Integer> blueHubTagIds = Set.of(27, 18, 25, 26, 24, 21, 20, 19);

  // Hub center field positions (Blue alliance WPILib origin, meters)
  // Red hub is at approx x=4.49, y=4.02 (center of red reef)
  // Blue hub is at approx x=13.06, y=4.02 (center of blue reef) 
  // TODO: Verify these with the official 2026 field drawing
  public static final Translation2d RED_HUB_POSITION  = new Translation2d(4.49,  4.02);
  public static final Translation2d BLUE_HUB_POSITION = new Translation2d(13.06, 4.02);
}
