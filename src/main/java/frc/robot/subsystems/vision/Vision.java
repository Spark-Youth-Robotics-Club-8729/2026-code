package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  /** Functional interface to match your Drive::addVisionMeasurement signature */
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
  }

  @AutoLog
  public static class VisionInputs {
    public boolean connected = false;
    public int tagCount = 0;
    public double tx = 0.0;
    public double ty = 0.0;
    public double avgTagDist = 0.0;
    public int[] visibleIds = new int[0];
    public Pose2d mt2Pose = new Pose2d();
    public double timestamp = 0.0;
    public boolean enabledVisionUpdatesPose = false; // use this to toggle whether vision data should be fed into the pose estimator or not
  }

  private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();    // DO NOT WORRY ABOUT RED ERRORS from this line and the inputs variable. its created once code is built, so it will work
  private final Supplier<Rotation2d> gyroSupplier;
  private final BiConsumer<Pose2d, Double>
      odometryConsumer; // Ri3D used this to give both the Pose and the timestamp

  public Vision(Supplier<Rotation2d> gyroSupplier, BiConsumer<Pose2d, Double> odometryConsumer) {
    this.gyroSupplier = gyroSupplier;
    this.odometryConsumer = odometryConsumer;

    // LL4 will now only use the unblocked ids (we pass the validIDs and ignore the blocked ones)
    int[] validIds =
        LAYOUT.getTags().stream()
            .mapToInt(tag -> tag.ID)
            .filter(id -> !BLOCKED_TAG_IDS.contains(id))
            .toArray();
    LimelightHelpers.SetFiducialIDFiltersOverride(CAMERA_NAME, validIds);

    // Seed the LL IMU on startup (means to set the LL4 IMU to the robot gyro)
    LimelightHelpers.SetIMUMode(
        CAMERA_NAME, 1); // we do this because LL4 has an IMU, while older versions dont
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Vision", inputs);

    // update Odometry using MegaTag 2 ONLY if enabled and data is valid
    if (inputs.enabledVisionUpdatesPose && inputs.tagCount >= MT2_MIN_TAGS && inputs.connected) {
      // Use standard deviation because it will prevetn the vision from jumping
      double xStdDev = 0.1 * Math.pow(inputs.avgTagDist, 2) / inputs.tagCount;
      double yStdDev = 0.1 * Math.pow(inputs.avgTagDist, 2) / inputs.tagCount;
      double thetaStdDev = 0.5; // trust the rboto gyro  mainly

      odometryConsumer.accept(
          inputs.mt2Pose, 
          inputs.timestamp, 
          VecBuilder.fill(xStdDev, yStdDev, thetaStdDev)
      );
    }

    // the below stuff is for the IMU of the LL4 itself (older limelights dont have IMUs but LL4 does, which is why this is needed)
    if (DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode(
          CAMERA_NAME, 1); // Seed (means to set the LL4 IMU to the robot gyro)
    } else { // when enabled
      LimelightHelpers.SetIMUMode(
          CAMERA_NAME, 4); // Internal + Assist (means uses both robot gyro and LL4 IMU)
    }
  }

  private void updateInputs() {
    // Get Gyro from robot and feed it to LL4
    double yaw = gyroSupplier.get().getDegrees(); // get robot's gyro yaw
    LimelightHelpers.SetRobotOrientation(CAMERA_NAME, yaw,0, 0, 0, 0, 0); // sets the robot orientation with the yaw. the pitch, roll, and rate are 0 because they dont change (unless robot is tipped over)
    var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CAMERA_NAME); // since yaw is given, it is easy for LL4 to calcualte the x,y pose of robot

    // Get all of the data from limelight and check if its valid, then update inputs
    inputs.connected = LimelightHelpers.getTV(CAMERA_NAME); // true if valid target is found
    inputs.tx = LimelightHelpers.getTX(CAMERA_NAME);
    inputs.ty = LimelightHelpers.getTY(CAMERA_NAME);

    if (mt2 != null) {
      inputs.tagCount = mt2.tagCount;
      inputs.avgTagDist = mt2.avgTagDist;
      inputs.mt2Pose = mt2.pose;
      inputs.timestamp = mt2.timestampSeconds;

      inputs.visibleIds = new int[mt2.rawFiducials.length];
      for (int i = 0; i < mt2.rawFiducials.length; i++) {
        inputs.visibleIds[i] = mt2.rawFiducials[i].id;
      }
    }
  }

  /** Toggle whether vision data interferes with the Pose Estimator */
  public void enabledVisionUpdatesPose(boolean enabled) {
    inputs.enabledVisionUpdatesPose = enabled;
  }

  /** Simple check if any tag is visible */
  public boolean hasTarget() {
    return inputs.tagCount > 0;
  }

  /** Specifically checks if we see a hub tag for OUR alliance */
  public boolean hasHubTarget() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue); // uses OUR alliance
    Set<Integer> hubTags = (alliance == Alliance.Red) ? RED_HUB_TAGS : BLUE_HUB_TAGS;

    for (int id : inputs.visibleIds) {
      if (hubTags.contains(id)) return true;
    }
    return false;
  }

  /** Returns distance to nearest Hub Tag for shot calculation */
  public double getDistanceToHub() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Set<Integer> hubTags = (alliance == Alliance.Red) ? RED_HUB_TAGS : BLUE_HUB_TAGS;

    var raw = LimelightHelpers.getRawFiducials(CAMERA_NAME);
    double minFound = Double.NaN;

    if (raw == null || raw.length == 0)
      return minFound; // check if raw is null or empty first to avoid crashes

    for (var f : raw) {
      if (hubTags.contains(f.id)) {
        if (Double.isNaN(minFound) || f.distToRobot < minFound) {
          minFound = f.distToRobot;
        }
      }
    }
    return minFound;
  }
}
