package frc.robot.subsystems.Index;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  public final SparkMax leftindexermotor;

  public final SparkMax rightindexermotor;

  public IndexerSubsystem() {
    leftindexermotor =
        new SparkMax(Indexconstants.leftindexCanid, MotorType.kBrushless); // left one
    rightindexermotor = new SparkMax(Indexconstants.rightindexCanid, MotorType.kBrushless);
  }

  public void rotate() {
    leftindexermotor.set(Indexconstants.indexspeed);
    rightindexermotor.set(Indexconstants.indexspeed);
  }

  public void backwards() {
    leftindexermotor.set(-Indexconstants.indexspeed);
    rightindexermotor.set(-Indexconstants.indexspeed);
  }

  public void stoprotate() {
    leftindexermotor.set(0);
    rightindexermotor.set(0);
  }

  public void setvoltage(double voltage) {
    if (voltage > Indexconstants.indexvoltage) {
      voltage = Indexconstants.indexvoltage;
    }

    leftindexermotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Leftcurrent: ", leftindexermotor.getOutputCurrent());
    SmartDashboard.putNumber("rightcurrent: ", rightindexermotor.getOutputCurrent());
  }
}
