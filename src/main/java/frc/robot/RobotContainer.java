// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandJoystick;
import org.frc5587.lib.control.DeadbandXboxController;

import java.io.IOException;

import org.frc5587.lib.advanced.AddressableLEDController;
import org.frc5587.lib.auto.AutoPath;
import org.frc5587.lib.auto.RamseteCommandWrapper;
import org.frc5587.lib.auto.RamseteCommandWrapper.RamseteConstants;

import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.BunnyDumper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LEDConstants;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Controllers
    private final DeadbandJoystick joy = new DeadbandJoystick(0, 1.5);
    private final DeadbandXboxController xboxController = new DeadbandXboxController(1);
    // Subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    private final BunnyDumper bunnyDumper = new BunnyDumper();
    // Commands
    private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, joy::getY, () -> -joy.getXCurveDampened());
    // Auto commands
    private final Command ramseteTest = new RamseteCommandWrapper(drivetrain, new AutoPath("s path"), new RamseteConstants(AutoConstants.KS, AutoConstants.KV, AutoConstants.KA, AutoConstants.KP, 3, 3, AutoConstants.DRIVETRAIN_KINEMATICS)).resetOdometryOnStart(); 
    // Others
    private final AddressableLEDController ledController = new AddressableLEDController(LEDConstants.PWM_PORT, LEDConstants.LED_LENGTH);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // make drivetrain use arcadeDrive to drive
        drivetrain.setDefaultCommand(arcadeDrive);
        // Configure the button bindings
        configureButtonBindings();

        ledController.startLEDStepHandlerNotifier((Integer step, AddressableLEDBuffer buffer) -> {
            return ledController.stretchRainbow(50 * 10, step, buffer);
        }, 0.02);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    public void configureButtonBindings() {
        JoystickButton bButton = new JoystickButton(xboxController, XboxController.Button.kB.value);
        Trigger leftTrigger = new Trigger(() -> xboxController.getTrigger(Hand.kLeft));

        // when b button and left trigger are pressed together, extend the pistons. when the two buttons are released, retract the pistons
        bButton.and(leftTrigger).whenActive(bunnyDumper::extend, bunnyDumper).whenInactive(bunnyDumper::retract, bunnyDumper);
        
        // bButton.and(leftTrigger).whenInactive(bunnyDumper::retract, bunnyDumper);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return null
        return ramseteTest;
    }
}
