package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmUp extends CommandBase {
    private Arm arm;
    
    /**
     * This command is designed for auto, because an end condition was needed. It
     * moves the arm up and ends when its close enough
     * 
     * @param arm arm subsystem
     */
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