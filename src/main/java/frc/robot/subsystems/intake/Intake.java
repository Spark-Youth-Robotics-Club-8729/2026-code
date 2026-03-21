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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    DOWN,
    JITTER
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
  // Detect bumper contact (stall) and stop driving the slapdown PID when we hit a hard stop
  private final Debouncer slapdownStallDebouncer =
      new Debouncer(slapdownStallDebounceSec, DebounceType.kRising);

  private final edu.wpi.first.wpilibj.Timer jitterTimer =
      new edu.wpi.first.wpilibj.Timer(); // jitter timer

  // Tunable PID gains (initialized from constants, then optionally overridden via Shuffleboard)
  private double upKp = slapdownUpKp;
  private double upKd = slapdownUpKd;
  private double downKp = slapdownDownKp;
  private double downKd = slapdownDownKd;
  private double jitterKp = slapdownJitterKp;
  private double jitterKd = slapdownJitterKd;

  // Shuffleboard tuning entries
  private final SendableChooser<SlapdownGoal> tuningModeChooser = new SendableChooser<>();
  private final GenericEntry tuningKpEntry;
  private final GenericEntry tuningKdEntry;
  private SlapdownGoal lastTuningMode = SlapdownGoal.UP;

  @AutoLogOutput private SlapdownGoal slapdownGoal = SlapdownGoal.UP;
  @AutoLogOutput private RollerGoal rollerGoal = RollerGoal.STOP;
  @AutoLogOutput private SlapdownState slapdownState = SlapdownState.MOVING;

  // ---------------------------------------------------------------------------
  // Constructor
  // ---------------------------------------------------------------------------

  public Intake(IntakeIO io) {
    this.io = io;

    // Shuffleboard setup for live slapdown PID tuning
    tuningModeChooser.setDefaultOption("Up", SlapdownGoal.UP);
    tuningModeChooser.addOption("Down", SlapdownGoal.DOWN);
    tuningModeChooser.addOption("Jitter", SlapdownGoal.JITTER);

    ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    tab.add("Slapdown PID Mode", tuningModeChooser).withPosition(0, 0).withSize(2, 1);
    tuningKpEntry = tab.add("Slapdown kP", upKp).withPosition(0, 1).withSize(1, 1).getEntry();
    tuningKdEntry = tab.add("Slapdown kD", upKd).withPosition(1, 1).withSize(1, 1).getEntry();
  }

  // ---------------------------------------------------------------------------
  // Periodic
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Allow live PID tuning from Shuffleboard before applying outputs
    updateTuningFromShuffleboard();

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
    outputs.slapdownMode = IntakeIOOutputMode.CLOSED_LOOP;

    switch (slapdownGoal) {
      case UP -> {
        outputs.kP = upKp;
        outputs.kD = upKd;
        outputs.slapdownPositionRad = slapdownUpAngleRad;
      }
      case DOWN -> {
        outputs.kP = downKp;
        outputs.kD = downKd;
        outputs.slapdownPositionRad = slapdownDownAngleRad;
      }
      case JITTER -> {
        outputs.kP = jitterKp;
        outputs.kD = jitterKd;
        // double radPerSec = jitterFrequencyHz * (2.0 * Math.PI);
        // double offset =
        //    Math.abs(
        //        Units.degreesToRadians(jitterAmplitudeDeg)
        //            * Math.sin(jitterTimer.get() * radPerSec));
        double offset;
        // If sine wave thingy does not work, then comment the two lines above, and uncomment this
        // one
        double cycleTime = 1.0 / jitterFrequencyHz;
        if ((jitterTimer.get() % cycleTime) < (cycleTime / 2.0)) {
          offset = Units.degreesToRadians(jitterAmplitudeDeg);
        } else {
          offset = 0.0;
        }
        outputs.slapdownPositionRad = slapdownDownAngleRad - offset;
      }
    }
    ;
    outputs.slapdownVelocityRadPerSec = 0.0;

    // ---- Stall detection ----
    boolean slapdownStalled =
        slapdownStallDebouncer.calculate(
            Math.abs(inputs.slapdownAppliedVolts) >= slapdownStallAppliedVolts
                && inputs.slapdownCurrentAmps >= slapdownStallCurrentAmps
                && Math.abs(inputs.slapdownVelocityRadPerSec) <= slapdownStallVelocityRadPerSec);

    if (slapdownStalled) {
      outputs.slapdownMode = IntakeIOOutputMode.BRAKE; // stop driving the PID at the hard stop
      outputs.slapdownVolts = 0.0;
    }

    // ---- Slapdown state ----
    boolean atGoal =
        slapdownStalled
            || Math.abs(inputs.slapdownPositionRad - outputs.slapdownPositionRad)
                <= slapdownToleranceRad;
    boolean settled = slapdownSettleDebouncer.calculate(atGoal) || slapdownStalled;

    slapdownState =
        settled
            ? (slapdownGoal == SlapdownGoal.UP ? SlapdownState.UP : SlapdownState.DOWN)
            : SlapdownState.MOVING;

    io.applyOutputs(outputs);

    Logger.recordOutput("Intake/SlapdownAtGoal", atGoal);
    Logger.recordOutput("Intake/SlapdownSettled", settled);
    Logger.recordOutput("Intake/SlapdownStalled", slapdownStalled);
  }

  private void updateTuningFromShuffleboard() {
    SlapdownGoal selected = tuningModeChooser.getSelected();
    if (selected == null) {
      selected = lastTuningMode;
    }

    // When the selected mode changes, push that mode's current gains into the UI
    if (selected != lastTuningMode) {
      tuningKpEntry.setDouble(getKpFor(selected));
      tuningKdEntry.setDouble(getKdFor(selected));
      lastTuningMode = selected;
    }

    double newKp = tuningKpEntry.getDouble(getKpFor(selected));
    double newKd = tuningKdEntry.getDouble(getKdFor(selected));

    setKpFor(selected, newKp);
    setKdFor(selected, newKd);

    Logger.recordOutput("Intake/SlapdownTuningMode", selected.toString());
    Logger.recordOutput("Intake/SlapdownTunedKp", newKp);
    Logger.recordOutput("Intake/SlapdownTunedKd", newKd);
  }

  private double getKpFor(SlapdownGoal goal) {
    return switch (goal) {
      case UP -> upKp;
      case DOWN -> downKp;
      case JITTER -> jitterKp;
    };
  }

  private double getKdFor(SlapdownGoal goal) {
    return switch (goal) {
      case UP -> upKd;
      case DOWN -> downKd;
      case JITTER -> jitterKd;
    };
  }

  private void setKpFor(SlapdownGoal goal, double value) {
    switch (goal) {
      case UP -> upKp = value;
      case DOWN -> downKp = value;
      case JITTER -> jitterKp = value;
    }
  }

  private void setKdFor(SlapdownGoal goal, double value) {
    switch (goal) {
      case UP -> upKd = value;
      case DOWN -> downKd = value;
      case JITTER -> jitterKd = value;
    }
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

  public void toggleSlapdown() {
    if (isSlapdownUp()) {
      setSlapdownGoal(SlapdownGoal.DOWN);
      // setRollerGoal(RollerGoal.INTAKE);
    } else {
      setSlapdownGoal(SlapdownGoal.UP);
      // setRollerGoal(RollerGoal.STOP);
    }
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

  /** Lowers the slapdown arm only, without running the roller. */
  public Command slapdownDownCommand() {
    return Commands.runOnce(() -> setSlapdownGoal(SlapdownGoal.DOWN), this);
  }

  /**
   * Shakes the slapdown arm up and down slightly to agitate game pieces. Alternates between the
   * DOWN position and a 5-degree offset.
   */
  public Command jitterCommand() {
    return Commands.startEnd(
            () -> {
              jitterTimer.restart();
              setSlapdownGoal(SlapdownGoal.JITTER);
              setRollerGoal(RollerGoal.INTAKE);
            },
            () -> {
              setSlapdownGoal(SlapdownGoal.DOWN);
              setRollerGoal(RollerGoal.STOP);
              jitterTimer.stop();
            },
            this)
        .withName("JitterCommand");
  }
}
