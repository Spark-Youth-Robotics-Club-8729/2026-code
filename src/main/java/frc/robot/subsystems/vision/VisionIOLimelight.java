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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

/** IO implementation for a Limelight camera using MegaTag 2 with MegaTag 1 fallback. */
public class VisionIOLimelight implements VisionIO {
  private final NetworkTable table;
  private final DoubleArrayEntry botposeMegaTag1;
  private final DoubleArrayEntry botposeMegaTag2;
  private final DoubleArrayEntry rawFiducials;

  private static final int RAW_FIDUCIAL_STRIDE = 7;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight on NetworkTables.
   */
  public VisionIOLimelight(String name) {
    table = NetworkTableInstance.getDefault().getTable(name);
    botposeMegaTag1 = table.getDoubleArrayTopic("botpose_wpiblue").getEntry(new double[] {});
    botposeMegaTag2 = table.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry(new double[] {});
    rawFiducials = table.getDoubleArrayTopic("rawfiducials").getEntry(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double[] mt1Data = botposeMegaTag1.get(new double[] {});
    double[] mt2Data = botposeMegaTag2.get(new double[] {});
    inputs.connected = mt2Data.length >= 7 || mt1Data.length >= 7;

    // Read tag IDs and distances from raw fiducials
    double[] raw = rawFiducials.get(new double[] {});
    Set<Short> tagIds = new HashSet<>();
    List<Double> tagDistances = new LinkedList<>();
    int tagCount = raw.length / RAW_FIDUCIAL_STRIDE;
    for (int i = 0; i < tagCount; i++) {
      int base = i * RAW_FIDUCIAL_STRIDE;
      tagIds.add((short) (int) raw[base]);
      tagDistances.add(raw[base + 4]);

      // Update latest target observation from first tag
      if (i == 0) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(raw[base + 1]), // txnc
                Rotation2d.fromDegrees(raw[base + 2])); // tync
      }
    }

    // Save tag IDs to inputs
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }

    // Calculate average tag distance
    double avgDistance =
        tagDistances.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

    // Add pose observations
    List<PoseObservation> poseObservations = new LinkedList<>();

    // MegaTag 2 — gyro-fused, stable translation, no usable rotation data
    if (mt2Data.length >= 7 && tagCount > 0) {
      poseObservations.add(
          new PoseObservation(
              Timer.getFPGATimestamp() - (mt2Data[6] / 1000.0), // Timestamp
              parsePose(mt2Data), // 3D pose estimate
              0.0, // Ambiguity (not applicable for MegaTag 2)
              tagCount, // Tag count
              avgDistance, // Average tag distance
              PoseObservationType.MEGATAG_2)); // Observation type
    }

    // MegaTag 1 fallback — full 3D solve, used only when MegaTag 2 is unavailable
    if (mt2Data.length < 7 && mt1Data.length >= 7 && tagCount > 0) {
      poseObservations.add(
          new PoseObservation(
              Timer.getFPGATimestamp() - (mt1Data[6] / 1000.0), // Timestamp
              parsePose(mt1Data), // 3D pose estimate
              raw.length >= RAW_FIDUCIAL_STRIDE ? raw[6] : 0.0, // Ambiguity
              tagCount, // Tag count
              avgDistance, // Average tag distance
              PoseObservationType.MEGATAG_1)); // Observation type
    }

    // Save pose observations to inputs object
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
  }

  /** Parses a Limelight botpose array into a Pose3d. */
  private static Pose3d parsePose(double[] data) {
    return new Pose3d(
        new Translation3d(data[0], data[1], data[2]),
        new Rotation3d(
            Math.toRadians(data[3]),
            Math.toRadians(data[4]),
            Math.toRadians(data[5])));
  }
}