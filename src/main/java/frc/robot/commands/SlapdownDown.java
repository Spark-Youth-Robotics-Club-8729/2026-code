package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class SlapdownDown extends Command {
  private final IntakeSubsystem intake;

  public SlapdownDown(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.setSlapdownManual(IntakeConstants.kSlapdownDownSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopSlapdown();
  }
}

