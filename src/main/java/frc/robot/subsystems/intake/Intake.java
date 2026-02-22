// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputMode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  // ---------------------------------------------------------------------------
  // Enums
  // ---------------------------------------------------------------------------

  public enum SlapdownGoal {
    UP,
    DOWN
  }

  public enum RollerGoal {
    INTAKE,
    OUTTAKE,
    STOP
  }

  public enum SlapdownState {
    UP,
    DOWN,
    MOVING
  }

  // ---------------------------------------------------------------------------
  // IO
  // ---------------------------------------------------------------------------

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO.IntakeIOOutputs outputs = new IntakeIO.IntakeIOOutputs();

  // ---------------------------------------------------------------------------
  // Alerts
  // ---------------------------------------------------------------------------

  private final Alert rollerDisconnected =
      new Alert("Intake roller motor disconnected!", Alert.AlertType.kWarning);
  private final Alert slapdownDisconnected =
      new Alert("Intake slapdown motor disconnected!", Alert.AlertType.kWarning);

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------

  // Arm must stay within tolerance for 150ms before we declare it settled,
  // preventing false positives from encoder noise mid-movement.
  private final Debouncer slapdownSettleDebouncer = new Debouncer(0.15, DebounceType.kRising);

  @AutoLogOutput private SlapdownGoal slapdownGoal = SlapdownGoal.UP;
  @AutoLogOutput private RollerGoal rollerGoal = RollerGoal.STOP;
  @AutoLogOutput private SlapdownState slapdownState = SlapdownState.MOVING;

  // ---------------------------------------------------------------------------
  // Constructor
  // ---------------------------------------------------------------------------

  public Intake(IntakeIO io) {
    this.io = io;
  }

  // ---------------------------------------------------------------------------
  // Periodic
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    rollerDisconnected.set(!inputs.rollerConnected);
    slapdownDisconnected.set(!inputs.slapdownConnected);

    // Brake everything when disabled
    if (DriverStation.isDisabled()) {
      outputs.slapdownMode = IntakeIOOutputMode.BRAKE;
      outputs.rollerVolts = 0.0;
      io.applyOutputs(outputs);
      return;
    }

    // ---- Roller ----
    outputs.rollerVolts =
        switch (rollerGoal) {
          case INTAKE -> rollerIntakeVolts;
          case OUTTAKE -> rollerOuttakeVolts;
          case STOP -> 0.0;
        };

    // ---- Slapdown ----
    outputs.kP = slapdownKp;
    outputs.kD = slapdownKd;
    outputs.slapdownMode = IntakeIOOutputMode.CLOSED_LOOP;
    outputs.slapdownPositionRad =
        switch (slapdownGoal) {
          case UP -> slapdownUpAngleRad;
          case DOWN -> slapdownDownAngleRad;
        };
    outputs.slapdownVelocityRadPerSec = 0.0;

    // ---- Slapdown state ----
    boolean atGoal =
        Math.abs(inputs.slapdownPositionRad - outputs.slapdownPositionRad) <= slapdownToleranceRad;
    boolean settled = slapdownSettleDebouncer.calculate(atGoal);

    slapdownState =
        settled
            ? (slapdownGoal == SlapdownGoal.UP ? SlapdownState.UP : SlapdownState.DOWN)
            : SlapdownState.MOVING;

    io.applyOutputs(outputs);

    Logger.recordOutput("Intake/SlapdownAtGoal", atGoal);
    Logger.recordOutput("Intake/SlapdownSettled", settled);
  }

  // ---------------------------------------------------------------------------
  // State queries
  // ---------------------------------------------------------------------------

  public boolean isSlapdownUp() {
    return slapdownState == SlapdownState.UP;
  }

  public boolean isSlapdownDown() {
    return slapdownState == SlapdownState.DOWN;
  }

  public boolean isSlapdownMoving() {
    return slapdownState == SlapdownState.MOVING;
  }

  // ---------------------------------------------------------------------------
  // Goal setters
  // ---------------------------------------------------------------------------

  public void setSlapdownGoal(SlapdownGoal goal) {
    this.slapdownGoal = goal;
  }

  public void setRollerGoal(RollerGoal goal) {
    this.rollerGoal = goal;
  }

  // ---------------------------------------------------------------------------
  // Commands
  // ---------------------------------------------------------------------------

  /**
   * Deploys the slapdown and runs the roller simultaneously. Finishes once the arm has fully
   * settled in the down position.
   */
  public Command slapdownAndIntakeCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              setSlapdownGoal(SlapdownGoal.DOWN);
              setRollerGoal(RollerGoal.INTAKE);
            },
            this),
        Commands.waitUntil(this::isSlapdownDown));
  }

  /** Raises the slapdown and stops the roller. */
  public Command retractCommand() {
    return Commands.runOnce(
        () -> {
          setSlapdownGoal(SlapdownGoal.UP);
          setRollerGoal(RollerGoal.STOP);
        },
        this);
  }

  /** Runs the roller inward while held, stops on release. Arm position unchanged. */
  public Command intakeCommand() {
    return Commands.startEnd(
        () -> setRollerGoal(RollerGoal.INTAKE), () -> setRollerGoal(RollerGoal.STOP), this);
  }

  /** Runs the roller outward while held, stops on release. Arm position unchanged. */
  public Command outtakeCommand() {
    return Commands.startEnd(
        () -> setRollerGoal(RollerGoal.OUTTAKE), () -> setRollerGoal(RollerGoal.STOP), this);
  }
}
