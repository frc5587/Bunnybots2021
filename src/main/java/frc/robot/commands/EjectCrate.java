package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class EjectCrate extends CommandBase {
    private final Intake intake;
    private final Timer timer = new Timer();

    /**
     * This is a simple timer based command. It runs the intake in reverse for 0.5
     * seconds as we determined that was enough to consistently eject any crate.
     * 
     * @param intake intake subsystem
     */
    public EjectCrate(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.out();
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