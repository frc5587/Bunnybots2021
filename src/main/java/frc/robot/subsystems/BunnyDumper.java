package frc.robot.subsystems;

import org.frc5587.lib.subsystems.PistonControl;

import frc.robot.Constants.BunnyDumperConstants;

public class BunnyDumper extends PistonControl {
    public BunnyDumper() {
        super(BunnyDumperConstants.PISTON_PORTS); // passes the variable in constants to the constructor in java-libs piston control
    }
}