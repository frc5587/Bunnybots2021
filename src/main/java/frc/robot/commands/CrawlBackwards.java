package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperSimpleDrivetrain;

public class CrawlBackwards extends CommandBase {
    private final SuperSimpleDrivetrain drivetrain;
    public CrawlBackwards(SuperSimpleDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.crawlBackwards();
    }
    @Override
    public void execute() {
        drivetrain.crawlBackwards();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}