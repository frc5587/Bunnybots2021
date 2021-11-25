// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandJoystick;
import org.frc5587.lib.control.DeadbandXboxController;
import org.frc5587.lib.advanced.AddressableLEDController;

import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

import frc.robot.Constants.LEDConstants;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

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
    private final DeadbandXboxController xb = new DeadbandXboxController(1);
    // Subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    // Commands
    private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, joy::getY, () -> -joy.getXCurveDampened());
    // Others
    private final AddressableLEDController ledController = new AddressableLEDController(LEDConstants.PWM_PORT,
            LEDConstants.LED_LENGTH);

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
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton yButton = new JoystickButton(xb, XboxController.Button.kY.value);
    Trigger leftTrigger = new Trigger(() -> xb.getTrigger(Hand.kLeft));

    // when y button is active, move intake forwards | when the y button is inactive - stop.
    yButton.whenActive(intake::forward, intake).whenInactive(intake::stop, intake)
    // when y button & left trigger are active, move intake backwards | when the y button & left trigger are inactive - stop. 
    yButton.and(leftTrigger).whenActive(intake::backward, intake).whenInactive(intake::stop, intake);
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
