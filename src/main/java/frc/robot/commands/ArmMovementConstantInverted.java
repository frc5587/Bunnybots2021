package frc.robot.commands;

import frc.robot.subsystems.Arm;

public class ArmMovementConstantInverted extends ArmMovementConstant {
    private final Arm arm;
    public ArmMovementConstantInverted(Arm arm) {
        super(arm);
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.moveArmFixedReversed();
    }
}
