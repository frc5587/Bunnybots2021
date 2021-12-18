package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class EjectCrate extends CommandBase {
    private final Intake intake;
    private final Timer timer = new Timer();

    public EjectCrate(Intake intake) {   
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.backward();
        timer.reset();
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(IntakeConstants.EJECT_CRATE_RUNTIME);
    }
}