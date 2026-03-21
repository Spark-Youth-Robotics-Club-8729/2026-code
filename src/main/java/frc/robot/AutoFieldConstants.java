// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.DriveConstants;
// import frc.robot.subsystems.shooter.ShotCalculator;

/**
 * Waypoints and poses used by autonomous routines.
 *
 * <p>All positions are in the WPILib field coordinate system (blue alliance origin). Robot width
 * offsets use {@link DriveConstants#fullWidthX} to keep the robot clear of obstacles.
 */
public class AutoFieldConstants {

  /** Broad zone categories used to describe where a waypoint sits on the field. */
  public enum Area {
    DEPOT,
    TOWER,
    OUTPOST,
    START,
    NEUTRAL_ZONE,
    LAUNCH
  }

  /**
   * A field waypoint: a 2-D translation paired with the zone it belongs to.
   *
   * <p>Use {@code new Waypoint(translation, area)} instead of a builder.
   */
  public record Waypoint(Translation2d translation, Area area) {}

  // ---------------------------------------------------------------------------
  // Starting positions
  // ---------------------------------------------------------------------------

  /** Starting positions at the trench openings. */
  public static class Trench {
    public static final Waypoint leftStart =
        new Waypoint(
            new Translation2d(
                FieldConstants.LinesVertical.starting - DriveConstants.fullWidthX / 2.0,
                FieldConstants.LinesHorizontal.leftTrenchOpenEnd + DriveConstants.fullWidthX / 2.0),
            Area.START);

    public static final Waypoint rightStart =
        new Waypoint(
            new Translation2d(
                FieldConstants.LinesVertical.starting - DriveConstants.fullWidthX / 2.0,
                FieldConstants.LinesHorizontal.rightTrenchOpenStart
                    - DriveConstants.fullWidthX / 2.0),
            Area.START);
  }

  /** Starting positions at the bump openings. */
  public static class Bump {
    public static final Waypoint leftInner =
        new Waypoint(
            new Translation2d(
                FieldConstants.LinesVertical.starting - DriveConstants.fullWidthX / 2.0,
                FieldConstants.LinesHorizontal.leftBumpMiddle),
            Area.START);

    public static final Waypoint rightInner =
        new Waypoint(
            new Translation2d(
                FieldConstants.LinesVertical.starting - DriveConstants.fullWidthX / 2.0,
                FieldConstants.LinesHorizontal.rightBumpMiddle),
            Area.START);

    public static final Waypoint leftOuter =
        new Waypoint(
            new Translation2d(
                FieldConstants.LinesVertical.neutralZoneNear + DriveConstants.fullWidthX / 2.0,
                FieldConstants.LinesHorizontal.leftBumpMiddle),
            Area.NEUTRAL_ZONE);

    public static final Waypoint rightOuter =
        new Waypoint(
            new Translation2d(
                FieldConstants.LinesVertical.neutralZoneNear + DriveConstants.fullWidthX / 2.0,
                FieldConstants.LinesHorizontal.rightBumpMiddle),
            Area.NEUTRAL_ZONE);
  }

  /** Starting position centered on the hub. */
  public static class Hub {
    public static final Waypoint centerStart =
        new Waypoint(
            new Translation2d(
                FieldConstants.LinesVertical.starting - DriveConstants.fullWidthX / 2.0,
                FieldConstants.LinesHorizontal.center),
            Area.START);
  }

  // ---------------------------------------------------------------------------
  // Depot
  // ---------------------------------------------------------------------------

  /** Waypoints for passing through the depot area. */
  public static class Depot {
    public static final Waypoint leftThrough =
        new Waypoint(
            new Translation2d(
                FieldConstants.Depot.depth / 2.0 + 0.2,
                FieldConstants.Depot.leftCorner.getY() + DriveConstants.fullWidthX / 2.0 + 0.1),
            Area.DEPOT);

    public static final Waypoint rightThrough =
        new Waypoint(
            new Translation2d(
                FieldConstants.Depot.depth / 2.0 + 0.2,
                FieldConstants.Depot.rightCorner.getY() - DriveConstants.fullWidthX / 2.0 - 0.1),
            Area.DEPOT);
  }

  // ---------------------------------------------------------------------------
  // Outpost
  // ---------------------------------------------------------------------------

  /** Intake poses at the outpost station. */
  public static class Outpost {
    /** Approach from the left side of the outpost. */
    public static final Pose2d leftIntake =
        new Pose2d(
            FieldConstants.Outpost.centerPoint.plus(
                new Translation2d(DriveConstants.fullWidthX / 2.0 + 0.1, 0.25)),
            Rotation2d.fromDegrees(-90));

    /** Approach from directly in front of the outpost. */
    public static final Pose2d frontIntake =
        new Pose2d(
            FieldConstants.Outpost.centerPoint.plus(
                new Translation2d(DriveConstants.fullWidthX / 2.0 + 0.25, 0.1)),
            Rotation2d.kPi);
  }

  // ---------------------------------------------------------------------------
  // Tower
  // ---------------------------------------------------------------------------

  /** Waypoints for routing past or to the tower. */
  public static class Tower {
    public static final Waypoint leftThrough =
        new Waypoint(
            new Translation2d(
                FieldConstants.Tower.frontFaceX / 2.0,
                FieldConstants.Tower.leftUpright.getY() + DriveConstants.fullWidthX / 2.0 + 0.25),
            Area.TOWER);

    public static final Waypoint rightThrough =
        new Waypoint(
            new Translation2d(
                FieldConstants.Tower.frontFaceX / 2.0,
                FieldConstants.Tower.rightUpright.getY() - DriveConstants.fullWidthX / 2.0 - 0.25),
            Area.TOWER);

    public static final Waypoint leftOutside =
        new Waypoint(
            FieldConstants.Tower.leftUpright.plus(
                new Translation2d(
                    DriveConstants.fullWidthX / 2.0 + 0.3, DriveConstants.fullWidthX / 2.0 + 0.3)),
            Area.TOWER);

    public static final Waypoint rightOutside =
        new Waypoint(
            FieldConstants.Tower.rightUpright.plus(
                new Translation2d(
                    DriveConstants.fullWidthX / 2.0 + 0.3, -DriveConstants.fullWidthX / 2.0 - 0.3)),
            Area.TOWER);
  }

  // ---------------------------------------------------------------------------
  // Climb
  // ---------------------------------------------------------------------------

  /** Poses for climbing the tower rungs. */
  public static class Climb {
    public static final Pose2d right =
        new Pose2d(
            FieldConstants.Tower.rightUpright.plus(
                new Translation2d(0.0, -DriveConstants.fullWidthX / 2.0)),
            Rotation2d.kPi);

    public static final Pose2d rightOffset =
        new Pose2d(
            FieldConstants.Tower.rightUpright.plus(
                new Translation2d(0.0, -DriveConstants.fullWidthX / 2.0 - 0.6)),
            Rotation2d.kPi);

    public static final Pose2d left =
        new Pose2d(
            FieldConstants.Tower.leftUpright.plus(
                new Translation2d(0.0, DriveConstants.fullWidthX / 2.0)),
            Rotation2d.kZero);

    public static final Pose2d leftOffset =
        new Pose2d(
            FieldConstants.Tower.leftUpright.plus(
                new Translation2d(0.0, DriveConstants.fullWidthX / 2.0 + 0.6)),
            Rotation2d.kZero);
  }

  // ---------------------------------------------------------------------------
  // Launch (pre-computed aimed shoot poses)
  // ---------------------------------------------------------------------------

  //   /** Pre-computed aimed poses for stationary shots during auto. */
  //   public static class Launch {
  //     public static final Pose2d leftTower =
  //         ShotCalculator.getStationaryAimedPose(
  //             Climb.left.getTranslation().plus(new Translation2d(1.5, 0.5)), true);

  //     public static final Pose2d rightTower =
  //         ShotCalculator.getStationaryAimedPose(
  //             Climb.right.getTranslation().plus(new Translation2d(1.5, -0.5)), true);

  //     public static Pose2d leftBump =
  //         ShotCalculator.getStationaryAimedPose(
  //             new Translation2d(
  //                 FieldConstants.LinesVertical.starting - 1.0,
  //                 FieldConstants.LinesHorizontal.leftBumpMiddle),
  //             true);

  //     public static Pose2d rightBump =
  //         ShotCalculator.getStationaryAimedPose(
  //             new Translation2d(
  //                 FieldConstants.LinesVertical.starting - 1.0,
  //                 FieldConstants.LinesHorizontal.rightBumpMiddle),
  //             true);
  //   }
}
