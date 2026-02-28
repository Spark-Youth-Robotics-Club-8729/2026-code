// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/**
 * VisionIO implementation for Limelight 4.
 *
 * <p>Uses LimelightHelpers for all NT communication. Runs both MegaTag1 (multi-tag, full 3D solve)
 * and MegaTag2 (gyro-fused, higher accuracy). LL4 internal IMU is seeded from the robot gyro while
 * disabled, then switched to internal+assist mode when enabled.
 */
public class VisionIOLimelight implements VisionIO {
  private final String name;
  private final Supplier<Rotation2d> rotationSupplier;

  // Heartbeat tracking for connection detection
  private double lastHeartbeat = -1.0;
  private long lastHeartbeatChangeMs = 0;

  /**
   * @param name Limelight name as configured in the web UI (e.g. "limelight").
   * @param rotationSupplier Supplier for the robot's current yaw (for MegaTag2 & IMU assist).
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    this.name = name;
    this.rotationSupplier = rotationSupplier;
  }

  /**
   * Call once while disabled to seed the LL4 internal IMU from the robot gyro and put the camera in
   * the correct IMU mode.
   */
  public void seedIMU() {
    LimelightHelpers.SetIMUMode(name, imuModeDisabled);
  }

  /**
   * Call once when the robot transitions to enabled to switch the LL4 to internal+assist IMU mode.
   */
  public void enableIMUAssist() {
    LimelightHelpers.SetIMUMode(name, imuModeEnabled);
    LimelightHelpers.SetIMUAssistAlpha(name, imuAssistAlpha);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // -----------------------------------------------------------------------
    // Connection monitoring via heartbeat
    // -----------------------------------------------------------------------
    double heartbeat = LimelightHelpers.getHeartbeat(name);
    long nowMs = System.currentTimeMillis();
    if (heartbeat != lastHeartbeat) {
      lastHeartbeat = heartbeat;
      lastHeartbeatChangeMs = nowMs;
    }
    inputs.connected = (nowMs - lastHeartbeatChangeMs) < heartbeatTimeoutMs;

    // -----------------------------------------------------------------------
    // Basic targeting data (tx/ty from crosshair — for servoing/auto-aim)
    // -----------------------------------------------------------------------
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(LimelightHelpers.getTX(name)),
            Rotation2d.fromDegrees(LimelightHelpers.getTY(name)));

    // -----------------------------------------------------------------------
    // Push robot orientation to LL4 for MegaTag2 + IMU assist
    // Must be called before getBotPoseEstimate_wpiBlue_MegaTag2
    // -----------------------------------------------------------------------
    double yawDeg = rotationSupplier.get().getDegrees();
    LimelightHelpers.SetRobotOrientation(name, yawDeg, 0.0, 0.0, 0.0, 0.0, 0.0);

    // -----------------------------------------------------------------------
    // Raw fiducial data — for logging and range-gating
    // -----------------------------------------------------------------------
    RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(name);
    Set<Integer> tagIdSet = new LinkedHashSet<>();
    List<Double> rawDistances = new ArrayList<>();
    double totalDist = 0.0;
    for (RawFiducial f : rawFiducials) {
      tagIdSet.add(f.id);
      rawDistances.add(f.distToRobot);
      totalDist += f.distToRobot;
    }
    inputs.tagIds = tagIdSet.stream().mapToInt(Integer::intValue).toArray();
    inputs.rawFiducialDistances = rawDistances.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.tagCount = rawFiducials.length;
    inputs.avgTagDistance = rawFiducials.length > 0 ? totalDist / rawFiducials.length : 0.0;

    // -----------------------------------------------------------------------
    // Read LL-provided standard deviations
    // stddevs = [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1yaw,
    //            MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
    // -----------------------------------------------------------------------
    double[] llStdDevs =
        NetworkTableInstance.getDefault()
            .getTable(name)
            .getEntry("stddevs")
            .getDoubleArray(new double[0]);

    // MT1: indices 0(x), 1(y), 5(yaw)
    double[] mt1StdDevs = null;
    if (llStdDevs.length >= 6) {
      mt1StdDevs = new double[] {llStdDevs[0], llStdDevs[1], llStdDevs[5]};
    }

    // MT2: indices 6(x), 7(y), 11(yaw)
    double[] mt2StdDevs = null;
    if (llStdDevs.length >= 12) {
      mt2StdDevs = new double[] {llStdDevs[6], llStdDevs[7], llStdDevs[11]};
    }

    // -----------------------------------------------------------------------
    // Pose observations — collect MegaTag1 and MegaTag2
    // -----------------------------------------------------------------------
    List<PoseObservation> observations = new ArrayList<>();

    // MegaTag1
    var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    if (mt1 != null && mt1.tagCount >= megatag1MinTags && mt1.pose != null) {
      double ambiguity =
          (mt1.rawFiducials != null && mt1.rawFiducials.length > 0)
              ? mt1.rawFiducials[0].ambiguity
              : 0.0;
      observations.add(
          new PoseObservation(
              mt1.timestampSeconds,
              new Pose3d(mt1.pose),
              ambiguity,
              mt1.tagCount,
              mt1.avgTagDist,
              PoseObservationType.MEGATAG_1,
              mt1StdDevs));
    }

    // MegaTag2
    var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if (mt2 != null && mt2.tagCount >= megatag2MinTags && mt2.pose != null) {
      observations.add(
          new PoseObservation(
              mt2.timestampSeconds,
              new Pose3d(mt2.pose),
              0.0,
              mt2.tagCount,
              mt2.avgTagDist,
              PoseObservationType.MEGATAG_2,
              mt2StdDevs));
    }

    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
  }
}
