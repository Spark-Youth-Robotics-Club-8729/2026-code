package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class SlapdownUp extends Command {
  private final IntakeSubsystem intake;

  public SlapdownUp(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.setSlapdownSpeed(IntakeConstants.kSlapdownUpSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopSlapdown();
  }
}
