// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  // Track last DriverStation state for IMU mode transitions
  private boolean wasDisabled = true;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " (Limelight 4) is disconnected.", AlertType.kWarning);
    }

    // Seed the LL4 internal IMU immediately on construction (robot starts disabled)
    for (var vio : io) {
      if (vio instanceof VisionIOLimelight ll) {
        ll.seedIMU();
      }
    }
  }

  /**
   * Returns the X angle to the best target from the specified camera index. Useful for simple
   * servoing (e.g. auto-aim while driving).
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  /** Returns the Y angle to the best target from the specified camera index. */
  public Rotation2d getTargetY(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.ty();
  }

  /** Returns whether the specified camera has any valid targets. */
  public boolean hasTarget(int cameraIndex) {
    return inputs[cameraIndex].tagCount > 0;
  }

  /** Returns the average tag distance for camera 0, useful for range estimation. */
  public double getAvgTagDistance(int cameraIndex) {
    return inputs[cameraIndex].avgTagDistance;
  }

  /**
   * Returns the average distance (meters) to visible hub AprilTags for the current alliance. Only
   * considers tags belonging to the scoring hub the robot is shooting at. Returns {@link
   * Double#NaN} if no relevant hub tags are visible.
   *
   * @param cameraIndex Camera to query.
   */
  public double getDistanceToHub(int cameraIndex) {
    var alliance = DriverStation.getAlliance();
    var hubIds =
        (alliance.isPresent() && alliance.get() == Alliance.Red)
            ? VisionConstants.redHubTagIds
            : VisionConstants.blueHubTagIds;

    int[] tagIds = inputs[cameraIndex].tagIds;
    double[] dists = inputs[cameraIndex].rawFiducialDistances;

    double total = 0.0;
    int count = 0;
    for (int i = 0; i < tagIds.length && i < dists.length; i++) {
      if (hubIds.contains(tagIds[i])) {
        total += dists[i];
        count++;
      }
    }
    return count > 0 ? total / count : Double.NaN;
  }

  /** Returns true if the camera can currently see at least one hub tag for the current alliance. */
  public boolean hasHubTarget(int cameraIndex) {
    return !Double.isNaN(getDistanceToHub(cameraIndex));
  }

  @Override
  public void periodic() {
    // -----------------------------------------------------------------------
    // LL4 IMU mode transitions
    // -----------------------------------------------------------------------
    boolean isDisabled = DriverStation.isDisabled();
    if (isDisabled && !wasDisabled) {
      // Transitioned to disabled — re-seed the internal IMU
      for (var vio : io) {
        if (vio instanceof VisionIOLimelight ll) ll.seedIMU();
      }
    } else if (!isDisabled && wasDisabled) {
      // Transitioned to enabled — switch to internal+assist mode
      for (var vio : io) {
        if (vio instanceof VisionIOLimelight ll) ll.enableIMUAssist();
      }
    }
    wasDisabled = isDisabled;

    // -----------------------------------------------------------------------
    // Update inputs
    // -----------------------------------------------------------------------
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // -----------------------------------------------------------------------
    // Process each camera
    // -----------------------------------------------------------------------
    List<Pose3d> allTagPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
    List<Pose3d> allRobotPosesRejected = new ArrayList<>();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      List<Pose3d> tagPoses = new ArrayList<>();
      List<Pose3d> robotPoses = new ArrayList<>();
      List<Pose3d> robotPosesAccepted = new ArrayList<>();
      List<Pose3d> robotPosesRejected = new ArrayList<>();

      // Collect visible tag poses for visualization
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      for (var observation : inputs[cameraIndex].poseObservations) {
        boolean rejectPose =
            observation.tagCount() == 0
                || (observation.tagCount() == 1
                    && observation.type() == PoseObservationType.MEGATAG_1
                    && observation.ambiguity() > maxAmbiguity)
                || Math.abs(observation.pose().getZ()) > maxZError
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
          continue;
        }
        robotPosesAccepted.add(observation.pose());

        // Calculate std devs — prefer LL-provided values, fall back to distance²/tagCount formula
        double linearStdDev;
        double angularStdDev;

        if (observation.stdDevs() != null
            && observation.stdDevs().length >= 3
            && observation.stdDevs()[0] > 0.0) {
          // Use Limelight's own stddevs [x, y, yaw]
          linearStdDev = observation.stdDevs()[0]; // x ~= y for our purposes
          angularStdDev = observation.stdDevs()[2];
        } else {
          // Fallback: scale with distance² / tagCount
          double stdDevFactor =
              Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
          linearStdDev = linearStdDevBaseline * stdDevFactor * cameraStdDevFactor;
          angularStdDev = angularStdDevBaseline * stdDevFactor * cameraStdDevFactor;

          if (observation.type() == PoseObservationType.MEGATAG_2) {
            linearStdDev *= linearStdDevMegatag2Factor;
            angularStdDev *= angularStdDevMegatag2Factor;
          }
        }

        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Per-camera logging
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagCount", inputs[cameraIndex].tagCount);
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/AvgTagDistance", inputs[cameraIndex].avgTagDistance);
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/HasTarget", inputs[cameraIndex].tagCount > 0);

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Summary logging
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
