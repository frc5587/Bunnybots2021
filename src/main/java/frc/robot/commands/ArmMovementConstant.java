package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmMovementConstant extends CommandBase{
    private final Arm arm;

    public ArmMovementConstant(Arm arm) {
        addRequirements(arm);

        this.arm = arm;
    }

    @Override
        public void initialize() {
    }

    @Override
    public void execute() {
        arm.moveArmFixedSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
