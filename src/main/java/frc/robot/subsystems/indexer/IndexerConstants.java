// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

public class IndexerConstants {
  // CAN ID
  public static final int motorID = 22; // TODO: confirm CAN ID

  // CAN bus ("rio" for roboRIO bus, or CANivore name)
  public static final String canBus = "rio";

  // Gear ratio: 4:1 planetary cartridge = 4 motor rotations per 1 output rotation
  public static final double gearRatio = 4.0;

  // Voltages (positive = feed toward shooter, negative = reverse)
  public static final double feedVolts = 6.0; // TODO: Tune
  public static final double reverseVolts = -4.0; // TODO: Tune

  private IndexerConstants() {}
}
