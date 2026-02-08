package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
  private final IntakeSubsystem intake;

  public Intake(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.setRollerSpeed(IntakeConstants.kIntakeInSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopRollers();
  }
}
