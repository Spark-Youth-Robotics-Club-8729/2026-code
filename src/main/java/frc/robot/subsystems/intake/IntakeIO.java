// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Roller (Kraken X60)
    public boolean rollerConnected = false;
    public double rollerVelocityRPM = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
    public double rollerTempCelsius = 0.0;

    // Slapdown (SparkMax + built-in NEO encoder)
    public boolean slapdownConnected = false;
    public double slapdownPositionRad = 0.0;
    public double slapdownVelocityRadPerSec = 0.0;
    public double slapdownAppliedVolts = 0.0;
    public double slapdownCurrentAmps = 0.0;
    public double slapdownTempCelsius = 0.0;
  }

  public enum IntakeIOOutputMode {
    BRAKE,
    COAST,
    OPEN_LOOP,
    CLOSED_LOOP
  }

  public static class IntakeIOOutputs {
    // Roller
    public double rollerVolts = 0.0;

    // Slapdown
    public IntakeIOOutputMode slapdownMode = IntakeIOOutputMode.BRAKE;
    public double slapdownPositionRad = 0.0;
    public double slapdownVelocityRadPerSec = 0.0;
    public double slapdownVolts = 0.0; // used in OPEN_LOOP mode
    public double kP = 0.0;
    public double kD = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void applyOutputs(IntakeIOOutputs outputs) {}
}
