package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  public final SparkMax leftindexermotor;

  public final SparkMax rightindexermotor;

  public IndexerSubsystem() {
    leftindexermotor =
        new SparkMax(IndexerConstants.leftindexCanid, MotorType.kBrushless); // left one
    rightindexermotor = new SparkMax(IndexerConstants.rightindexCanid, MotorType.kBrushless);
  }

  public void rotate() {
    leftindexermotor.set(IndexerConstants.indexspeed);
    rightindexermotor.set(IndexerConstants.indexspeed);
  }

  public void backwards() {
    leftindexermotor.set(-IndexerConstants.indexspeed);
    rightindexermotor.set(-IndexerConstants.indexspeed);
  }

  public void stoprotate() {
    leftindexermotor.set(0);
    rightindexermotor.set(0);
  }

  public void setvoltage(double voltage) {
    if (voltage > IndexerConstants.indexvoltage) {
      voltage = IndexerConstants.indexvoltage;
    }

    leftindexermotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Leftcurrent: ", leftindexermotor.getOutputCurrent());
    SmartDashboard.putNumber("rightcurrent: ", rightindexermotor.getOutputCurrent());
  }
}
