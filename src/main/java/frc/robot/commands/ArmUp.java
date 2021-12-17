package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmUp extends CommandBase {
    private static Arm arm;
    
    public ArmUp(Arm arm) {
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.moveUp();
    }

    @Override
    public boolean isFinished() {
        return arm.getAngleRadians() > ArmConstants.HIGHER_MOVE_THRESHOLD;
    }
}