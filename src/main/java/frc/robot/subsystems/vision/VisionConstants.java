package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.util.Set;

public class VisionConstants {
  public static final String CAMERA_NAME = "limelight";

  public static final AprilTagFieldLayout LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Tags to ignore for pose estimation
  public static final Set<Integer> BLOCKED_TAG_IDS = Set.of(9, 25, 18, 5, 2, 21);

  // Hub Tags for scoring calculations
  public static final Set<Integer> RED_HUB_TAGS = Set.of(8, 10, 3, 4, 11);
  public static final Set<Integer> BLUE_HUB_TAGS = Set.of(27, 26, 24, 20, 19);

  // Trust MegaTag2 with 1 tag, MegaTag1 with 2+
  public static final int MT1_MIN_TAGS = 2;
  public static final int MT2_MIN_TAGS = 1;
}
