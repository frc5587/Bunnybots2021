package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BunnyDumperConstants;;

public class BunnyDumper extends SubsystemBase {
    private DoubleSolenoid piston1 = new DoubleSolenoid(BunnyDumperConstants.PISTON_PORTS[0], BunnyDumperConstants.PISTON_PORTS[1]);

    public BunnyDumper() {
        retract();
    }

    public void extend() {
        piston1.set(kForward);
    }

    public void retract() {
        piston1.set(kReverse);
    }
}