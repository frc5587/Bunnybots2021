package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmDown extends CommandBase {
    private Arm arm;

    /**
     * This command is designed for auto, because an end condition was needed. It
     * moves the arm down and ends when its close enough
     * 
     * @param arm arm subsystem
     */
    public ArmDown(Arm arm) {
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.moveDown();
    }

    @Override
    public boolean isFinished() {
        return arm.getAngleRadians() < ArmConstants.LOWER_MOVE_THRESHOLD;
    }
}