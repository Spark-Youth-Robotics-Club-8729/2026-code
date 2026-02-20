package main.java.frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    public final SparkMax leftindexermotor;

    public final SparkMax rightindexermotor;

    public IndexerSubsystem(){ 
        leftindexermotor = new SparkMax(IndexerConstants.leftindexCanid, MotorType.kBrushless); //left one 
        rightindexermotor = new SparkMax(IndexerConstants.rightindexCanid, MotorType.kBrushless);

    }

    public void rotate() {
        leftindexermotor.set(indexspeed);
        rightindexermotor.set(indexspeed);
    }
    
    public void backwards(){
        leftindexermotor.set(-indexerspeed);
        rightindexermotor.set(-indexerspeed);
    }

    public void stoprotate(){
        leftindexermotor.set(0);
        rightindexermotor.set(0);
    }
    public void setvoltage(double voltage){
        if (voltage>indexvoltage){
            voltage=indexvoltage;
        }

        leftindexermotor.setvoltage(voltage);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Leftcurrent: ", leftindexermotor.getOutputCurrent);
        SmartDashboard.putNumber("rightcurrent: ", rightindexermotor.getOutputCurrent);
    }

}
