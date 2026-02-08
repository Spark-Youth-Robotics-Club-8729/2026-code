package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class SlapdownAndIntake extends Command {
  private final IntakeSubsystem intake;

  public SlapdownAndIntake(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    // nothing
  }

  @Override
  public void execute() {
    intake.goToPosition(IntakeConstants.kDownPosition);
    intake.setRollerSpeed(IntakeConstants.kIntakeInSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopSlapdown();
    intake.stopRollers();
  }
}

