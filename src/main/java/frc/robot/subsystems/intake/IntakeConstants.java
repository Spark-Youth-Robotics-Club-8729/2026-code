// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {
  // CAN IDs
  public static final int rollerMotorID = 20; // TODO: Set actual CAN ID
  public static final int slapdownMotorID = 21; // TODO: Set actual CAN ID
  public static final String canBus = "rio"; // Change to "canivore" if needed

  // Slapdown gear ratio (motor rotations per mechanism rotation)
  public static final double slapdownGearRatio = 1.0; // TODO: Set actual gear ratio

  // Slapdown angle limits (radians, at the mechanism after gear reduction)
  public static final double slapdownUpAngleRad = Units.degreesToRadians(0.0); // TODO: Set actual
  public static final double slapdownDownAngleRad =
      Units.degreesToRadians(90.0); // TODO: Set actual

  // Slapdown PID gains
  public static final double slapdownKp = 0.0; // TODO: Tune
  public static final double slapdownKd = 0.5; // TODO: Tune

  // Slapdown tolerance
  public static final double slapdownToleranceRad = Units.degreesToRadians(2.0);

  // Roller voltages (positive = intake in, negative = outtake)
  public static final double rollerIntakeVolts = 10.0; // TODO: Tune
  public static final double rollerOuttakeVolts = -6.0; // TODO: Tune

  private IntakeConstants() {}
}
