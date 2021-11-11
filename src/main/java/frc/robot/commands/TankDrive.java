package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier leftThrottleSupplier, rightThrottleSupplier;

    // TankDrive uses input from two joysticks to drive each side of the robot.
    public TankDrive(Drivetrain drivetrain, DoubleSupplier leftThrottleSupplier, DoubleSupplier rightThrottleSupplier) {
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;
        this.leftThrottleSupplier = leftThrottleSupplier;
        this.rightThrottleSupplier = rightThrottleSupplier;
    }
    @Override
        public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var leftThrottle = leftThrottleSupplier.getAsDouble();
        var rightThrottle = rightThrottleSupplier.getAsDouble();
        drivetrain.tankDrive(leftThrottle, rightThrottle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
