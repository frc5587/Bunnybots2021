package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.FullArmSubsys;

public class ArmMovementThrottle extends CommandBase {
    private final FullArmSubsys arm;
    private final DoubleSupplier throttleSupplier;
    
    public ArmMovementThrottle(FullArmSubsys arm, DoubleSupplier throttleSupplier) {
        addRequirements(arm);

        this.arm = arm;
        this.throttleSupplier = throttleSupplier;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.set(throttleSupplier.getAsDouble());
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
