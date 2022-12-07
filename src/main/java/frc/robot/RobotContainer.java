// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.frc5587.lib.control.DeadbandJoystick;
import org.frc5587.lib.control.DeadbandXboxController;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DDRDrive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Controllers
        private final DeadbandJoystick joystick = new DeadbandJoystick(0, 1.5, 0.02);
        private final DeadbandXboxController xboxController = new DeadbandXboxController(1);

        // Subsystems
        private final Drivetrain drivetrain = new Drivetrain();

        // Commands
        private ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, () -> -joystick.getX(), () -> joystick.getY());
        private ArcadeDrive ddrArcadeDrive = new ArcadeDrive(drivetrain, buttonToValue(xboxController::getXButton, xboxController::getBButton, Constants.DrivetrainConstants.DDR_FWD), buttonToValue(xboxController::getYButton, xboxController::getAButton, Constants.DrivetrainConstants.DDR_TURN));
        private DDRDrive ddrDrive = new DDRDrive(drivetrain, buttonToValue(xboxController::getXButton, xboxController::getBButton, Constants.DrivetrainConstants.DDR_FWD), buttonToValue(xboxController::getYButton, xboxController::getAButton, Constants.DrivetrainConstants.DDR_TURN));

        // Other
        private NetworkTableEntry chooseDriveMode = SmartDashboard.getEntry("Drive Mode");

        SendableChooser<Command> driveChooser = new SendableChooser<Command>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                driveChooser.setDefaultOption("Arcade Drive", arcadeDrive);
                driveChooser.addOption("DDR Arcade Drive", ddrArcadeDrive);
                driveChooser.addOption("DDR Drive", ddrDrive);
                SmartDashboard.putData(driveChooser);

                drivetrain.setDefaultCommand(driveChooser.getSelected());
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
        }

        private static DoubleSupplier buttonToValue(BooleanSupplier pos, BooleanSupplier neg, double power) {
                return () -> {
                    if(pos.getAsBoolean() == neg.getAsBoolean()) return 0;
                    else if(pos.getAsBoolean()) return power;
                    else return -power;
                };
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return null;
        }
}