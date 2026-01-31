
package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {

    private final DoubleSolenoid doubleSolenoid;
    private final Compressor compressor;

    public PneumaticsSubsystem() {

        // Enable compressor on REV PH using pressure switch
        compressor = new Compressor(
            Constants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH
        );
        compressor.enableDigital();

        // Double solenoid setup
        doubleSolenoid = new DoubleSolenoid(
            Constants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH,
            Constants.EXTEND_SOLENOID_ID,
            Constants.RETRACT_SOLENOID_ID
        );
    }

    public void extend() {
        doubleSolenoid.set(Value.kForward);
    }

    public void retract() {
        doubleSolenoid.set(Value.kReverse);
    }

    public void disable() {
        doubleSolenoid.set(Value.kOff);
    }

    @Override
    public void periodic() {
        // nothing needed here
    }
}