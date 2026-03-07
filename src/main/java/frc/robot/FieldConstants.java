// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Field element locations and dimensions for the 2026 game.
 *
 * <p>All constants are in the WPILib field coordinate system (blue alliance origin, meters). The
 * field is viewed from above with the blue alliance on the left (low X) and red on the right (high
 * X).
 */
public class FieldConstants {

  // AprilTag layout — same game as VisionConstants but loaded here for position math
  static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final double aprilTagWidth = Units.inchesToMeters(6.5);

  // Field dimensions
  public static final double fieldLength = aprilTagLayout.getFieldLength();
  public static final double fieldWidth = aprilTagLayout.getFieldWidth();

  // Fuel ball diameter (2026 game piece)
  public static final double fuelDiameter = Units.inchesToMeters(5.91);

  // ---------------------------------------------------------------------------
  // Pre-computed hub near-face X positions.
  // Used by Hub, LinesVertical, and bump classes to avoid cross-class init
  // order issues between static nested classes.
  // Blue hub near face: derived from AprilTag 26 (blue hub, near face).
  // Red hub near face:  derived from AprilTag 4  (red hub, near face).
  // Fallbacks use the hardcoded hub center positions ± half hub width.
  // ---------------------------------------------------------------------------
  static final double BLUE_HUB_NEAR_X =
      aprilTagLayout
          .getTagPose(26)
          .map(p -> p.getX())
          .orElse(13.06 - Units.inchesToMeters(47.0) / 2.0);

  static final double RED_HUB_NEAR_X =
      aprilTagLayout
          .getTagPose(4)
          .map(p -> p.getX())
          .orElse(4.49 - Units.inchesToMeters(47.0) / 2.0);

  // ---------------------------------------------------------------------------
  // Vertical zone boundaries (X-axis)
  // ---------------------------------------------------------------------------

  /** Field X-axis reference lines. */
  public static class LinesVertical {
    public static final double center = fieldLength / 2.0;

    /** X of the blue alliance zone / starting line (= blue hub near face). */
    public static final double allianceZone = BLUE_HUB_NEAR_X;

    public static final double starting = allianceZone;

    /** X of the blue hub center. */
    public static final double hubCenter = BLUE_HUB_NEAR_X + Units.inchesToMeters(47.0) / 2.0;

    public static final double neutralZoneNear = center - Units.inchesToMeters(120);
    public static final double neutralZoneFar = center + Units.inchesToMeters(120);

    /** X of the red hub center. */
    public static final double oppHubCenter = RED_HUB_NEAR_X + Units.inchesToMeters(47.0) / 2.0;

    /** X of the red alliance zone (from tag 10 on red hub). */
    public static final double oppAllianceZone =
        aprilTagLayout.getTagPose(10).map(p -> p.getX()).orElse(RED_HUB_NEAR_X);
  }

  // ---------------------------------------------------------------------------
  // Horizontal zone boundaries (Y-axis)
  // Left = high Y (towards the left wall when facing from the blue alliance station)
  // Right = low Y (towards the right wall)
  // ---------------------------------------------------------------------------

  /** Field Y-axis reference lines. */
  public static class LinesHorizontal {
    public static final double center = fieldWidth / 2.0;

    // Shared half-width of the hub
    private static final double hubHalfWidth = Units.inchesToMeters(47.0) / 2.0;

    // Right of hub (low Y side)
    public static final double rightBumpStart = center - hubHalfWidth;
    public static final double rightBumpEnd = rightBumpStart - Units.inchesToMeters(73.0);
    public static final double rightBumpMiddle = (rightBumpStart + rightBumpEnd) / 2.0;
    public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
    public static final double rightTrenchOpenEnd = 0;

    // Left of hub (high Y side)
    public static final double leftBumpEnd = center + hubHalfWidth;
    public static final double leftBumpStart = leftBumpEnd + Units.inchesToMeters(73.0);
    public static final double leftBumpMiddle = (leftBumpStart + leftBumpEnd) / 2.0;
    public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
    public static final double leftTrenchOpenStart = fieldWidth;
  }

  // ---------------------------------------------------------------------------
  // Hub
  // ---------------------------------------------------------------------------

  /** Hub (scoring goal) physical dimensions and field positions. */
  public static class Hub {
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(72.0); // top of catcher
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    // --- Blue hub (alliance side) ---

    /** 3-D center of the top opening of the blue hub. */
    public static final Translation3d topCenterPoint =
        new Translation3d(BLUE_HUB_NEAR_X + width / 2.0, fieldWidth / 2.0, height);

    /** 3-D center of the inner scoring zone of the blue hub. */
    public static final Translation3d innerCenterPoint =
        new Translation3d(BLUE_HUB_NEAR_X + width / 2.0, fieldWidth / 2.0, innerHeight);

    // Blue hub 2-D corners (near = close to blue alliance, far = far side)
    public static final Translation2d nearLeftCorner =
        new Translation2d(BLUE_HUB_NEAR_X, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(BLUE_HUB_NEAR_X, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d farLeftCorner =
        new Translation2d(BLUE_HUB_NEAR_X + width, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d farRightCorner =
        new Translation2d(BLUE_HUB_NEAR_X + width, fieldWidth / 2.0 - width / 2.0);

    // Blue hub face poses (from AprilTags on each face)
    public static final Pose2d nearFace =
        aprilTagLayout.getTagPose(26).map(p -> p.toPose2d()).orElse(new Pose2d());
    public static final Pose2d farFace =
        aprilTagLayout.getTagPose(20).map(p -> p.toPose2d()).orElse(new Pose2d());
    public static final Pose2d rightFace =
        aprilTagLayout.getTagPose(18).map(p -> p.toPose2d()).orElse(new Pose2d());
    public static final Pose2d leftFace =
        aprilTagLayout.getTagPose(21).map(p -> p.toPose2d()).orElse(new Pose2d());

    // --- Red hub (opposing side) ---

    /** 3-D center of the top opening of the red hub. */
    public static final Translation3d oppTopCenterPoint =
        new Translation3d(RED_HUB_NEAR_X + width / 2.0, fieldWidth / 2.0, height);

    /** 3-D center of the inner scoring zone of the red hub. */
    public static final Translation3d oppInnerCenterPoint =
        new Translation3d(RED_HUB_NEAR_X + width / 2.0, fieldWidth / 2.0, innerHeight);

    // Red hub 2-D corners
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(RED_HUB_NEAR_X, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppNearRightCorner =
        new Translation2d(RED_HUB_NEAR_X, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(RED_HUB_NEAR_X + width, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppFarRightCorner =
        new Translation2d(RED_HUB_NEAR_X + width, fieldWidth / 2.0 - width / 2.0);
  }

  // ---------------------------------------------------------------------------
  // Bumps (obstacles flanking the hub)
  // ---------------------------------------------------------------------------

  /** Left bump (left of hub when facing from the blue alliance station, high-Y side). */
  public static class LeftBump {
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // Alliance side
    public static final Translation2d nearLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2.0, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
    public static final Translation2d farLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2.0, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner = Hub.farLeftCorner;
  }

  /** Right bump (right of hub when facing from the blue alliance station, low-Y side). */
  public static class RightBump {
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // Alliance side
    public static final Translation2d nearLeftCorner = Hub.nearRightCorner;
    public static final Translation2d nearRightCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2.0, Units.inchesToMeters(255));
    public static final Translation2d farLeftCorner = Hub.farRightCorner;
    public static final Translation2d farRightCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2.0, Units.inchesToMeters(255));
  }

  // ---------------------------------------------------------------------------
  // Trenches
  // ---------------------------------------------------------------------------

  /** Left trench (runs through the left-bump area into the hub). */
  public static class LeftTrench {
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    public static final Translation3d openingTopLeft =
        new Translation3d(LinesVertical.hubCenter, fieldWidth, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);
    public static final Translation3d oppOpeningTopLeft =
        new Translation3d(LinesVertical.oppHubCenter, fieldWidth, openingHeight);
    public static final Translation3d oppOpeningTopRight =
        new Translation3d(LinesVertical.oppHubCenter, fieldWidth - openingWidth, openingHeight);
  }

  /** Right trench (runs through the right-bump area into the hub). */
  public static class RightTrench {
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    public static final Translation3d openingTopLeft =
        new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(LinesVertical.hubCenter, 0, openingHeight);
    public static final Translation3d oppOpeningTopLeft =
        new Translation3d(LinesVertical.oppHubCenter, openingWidth, openingHeight);
    public static final Translation3d oppOpeningTopRight =
        new Translation3d(LinesVertical.oppHubCenter, 0, openingHeight);
  }

  // ---------------------------------------------------------------------------
  // Tower
  // ---------------------------------------------------------------------------

  /** Climbing tower structure. */
  public static class Tower {
    public static final double width = Units.inchesToMeters(49.25);
    public static final double depth = Units.inchesToMeters(45.0);
    public static final double height = Units.inchesToMeters(78.25);
    public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
    public static final double frontFaceX = Units.inchesToMeters(43.51);
    public static final double uprightHeight = Units.inchesToMeters(72.1);

    // Rung heights from the floor
    public static final double lowRungHeight = Units.inchesToMeters(27.0);
    public static final double midRungHeight = Units.inchesToMeters(45.0);
    public static final double highRungHeight = Units.inchesToMeters(63.0);

    // Blue tower — Y from tag 31
    private static final double blueTowerY =
        aprilTagLayout.getTagPose(31).map(p -> p.getY()).orElse(fieldWidth / 2.0);

    public static final Translation2d centerPoint = new Translation2d(frontFaceX, blueTowerY);
    public static final Translation2d leftUpright =
        new Translation2d(
            frontFaceX, blueTowerY + innerOpeningWidth / 2.0 + Units.inchesToMeters(0.75));
    public static final Translation2d rightUpright =
        new Translation2d(
            frontFaceX, blueTowerY - innerOpeningWidth / 2.0 - Units.inchesToMeters(0.75));

    // Red (opposing) tower — Y from tag 15
    private static final double redTowerY =
        aprilTagLayout.getTagPose(15).map(p -> p.getY()).orElse(fieldWidth / 2.0);

    public static final Translation2d oppCenterPoint =
        new Translation2d(fieldLength - frontFaceX, redTowerY);
    public static final Translation2d oppLeftUpright =
        new Translation2d(
            fieldLength - frontFaceX,
            redTowerY + innerOpeningWidth / 2.0 + Units.inchesToMeters(0.75));
    public static final Translation2d oppRightUpright =
        new Translation2d(
            fieldLength - frontFaceX,
            redTowerY - innerOpeningWidth / 2.0 - Units.inchesToMeters(0.75));
  }

  // ---------------------------------------------------------------------------
  // Depot
  // ---------------------------------------------------------------------------

  /** Fuel depot (fuel storage structure near the alliance wall). */
  public static class Depot {
    public static final double width = Units.inchesToMeters(42.0);
    public static final double depth = Units.inchesToMeters(27.0);
    public static final double height = Units.inchesToMeters(1.125);
    public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

    public static final Translation3d depotCenter =
        new Translation3d(depth, (fieldWidth / 2.0) + distanceFromCenterY, height);
    public static final Translation3d leftCorner =
        new Translation3d(depth, (fieldWidth / 2.0) + distanceFromCenterY + (width / 2.0), height);
    public static final Translation3d rightCorner =
        new Translation3d(depth, (fieldWidth / 2.0) + distanceFromCenterY - (width / 2.0), height);
  }

  // ---------------------------------------------------------------------------
  // Outpost
  // ---------------------------------------------------------------------------

  /** Outpost (fuel intake station at the alliance wall). */
  public static class Outpost {
    public static final double width = Units.inchesToMeters(31.8);
    public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
    public static final double height = Units.inchesToMeters(7.0);

    // Y from tag 29
    private static final double outpostY =
        aprilTagLayout.getTagPose(29).map(p -> p.getY()).orElse(fieldWidth / 2.0);

    public static final Translation2d centerPoint = new Translation2d(0, outpostY);
  }

  // ---------------------------------------------------------------------------
  // Fuel Pool
  // ---------------------------------------------------------------------------

  /** Center-field fuel pool. */
  public static class FuelPool {
    public static final double width = Units.inchesToMeters(181.9);
    public static final double depth = Units.inchesToMeters(71.9);

    public static final Translation2d nearLeftCorner =
        new Translation2d(fieldLength / 2.0 - depth / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(fieldLength / 2.0 - depth / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d leftCenter =
        new Translation2d(fieldLength / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d rightCenter =
        new Translation2d(fieldLength / 2.0, fieldWidth / 2.0 - width / 2.0);
  }
}
