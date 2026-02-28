// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  public enum IndexerGoal {
    FEED, // toward shooter
    REVERSE, // away from shooter
    STOP
  }

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final IndexerIO.IndexerIOOutputs outputs = new IndexerIO.IndexerIOOutputs();

  private final Alert motorDisconnected =
      new Alert("Indexer motor disconnected!", Alert.AlertType.kWarning);

  @AutoLogOutput private IndexerGoal goal = IndexerGoal.STOP;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    motorDisconnected.set(!inputs.connected);

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      outputs.volts = 0.0;
      io.applyOutputs(outputs);
      return;
    }

    outputs.volts =
        switch (goal) {
          case FEED -> feedVolts;
          case REVERSE -> reverseVolts;
          case STOP -> 0.0;
        };

    io.applyOutputs(outputs);
    Logger.recordOutput("Indexer/GoalVolts", outputs.volts);
  }

  // ---------------------------------------------------------------------------
  // Goal setter
  // ---------------------------------------------------------------------------

  public void setGoal(IndexerGoal goal) {
    this.goal = goal;
  }

  /** Feeds notes toward the shooter. Call from within a command that already requires this subsystem. */
  public void feed() {
    setGoal(IndexerGoal.FEED);
  }

  /** Stops the indexer. Call from within a command that already requires this subsystem. */
  public void stop() {
    setGoal(IndexerGoal.STOP);
  }

  // ---------------------------------------------------------------------------
  // Inline commands -- replaces IndexerCommands.java
  // ---------------------------------------------------------------------------

  /** Feeds notes toward the shooter while held, stops on release. */
  public Command feedCommand() {
    return Commands.startEnd(
        () -> setGoal(IndexerGoal.FEED), () -> setGoal(IndexerGoal.STOP), this);
  }

  /** Reverses the indexer while held, stops on release. */
  public Command reverseCommand() {
    return Commands.startEnd(
        () -> setGoal(IndexerGoal.REVERSE), () -> setGoal(IndexerGoal.STOP), this);
  }

  /** Stops the indexer immediately. */
  public Command stopCommand() {
    return Commands.runOnce(() -> setGoal(IndexerGoal.STOP), this);
  }
}
