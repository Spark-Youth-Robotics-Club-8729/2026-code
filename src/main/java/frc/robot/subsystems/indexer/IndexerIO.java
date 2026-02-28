// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected = false;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public static class IndexerIOOutputs {
    public double volts = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void applyOutputs(IndexerIOOutputs outputs) {}
}
