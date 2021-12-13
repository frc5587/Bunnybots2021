package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class EjectCrate extends CommandBase {
    private final Intake intake;

    public EjectCrate(Intake intake) {   
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.backward();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return intake.isCrateEjected();
    }
}