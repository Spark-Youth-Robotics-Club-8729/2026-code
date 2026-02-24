package frc.robot.subsystems.Index;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  public final SparkMax Indexmotor;

  public IndexerSubsystem() {
    Indexmotor = new SparkMax(Indexconstants.indexCanid, MotorType.kBrushless);
  }

  public void rotate() {
    Indexmotor.set(Indexconstants.indexspeed);
  }

  public void backwards() {
    Indexmotor.set(-Indexconstants.indexspeed);
  }

  public void stoprotate() {
    Indexmotor.set(0);
  }

  public void setvoltage(double voltage) {
    if (voltage > Indexconstants.indexvoltage) {
      voltage = Indexconstants.indexvoltage;
    }

    Indexmotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("indexcurrent: ", Indexmotor.getOutputCurrent());
  }
}
