package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

public class DDRDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier throttleSupplier;
    private final DoubleSupplier curveSupplier;

    public DDRDrive(Drivetrain drivetrain, DoubleSupplier throttleSupplier, DoubleSupplier curveSupplier) {
        this.drivetrain = drivetrain;
        this.throttleSupplier = throttleSupplier;
        this.curveSupplier = curveSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        var throttle = throttleSupplier.getAsDouble();
        var curve = curveSupplier.getAsDouble();
        drivetrain.arcadeDrive(throttle, curve);
        new WaitCommand(1);
        end(true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}