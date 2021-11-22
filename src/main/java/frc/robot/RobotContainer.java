// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.frc5587.lib.control.DeadbandXboxController;

import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // subsystems
  private final Intake intake = new Intake();
  // controllers
  private final DeadbandXboxController xb = new DeadbandXboxController(1);
  // commands
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
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

    // when y button is active, move intake forwards
    yButton.whenActive(intake::forward, intake);
    // when y button & left trigger are active, move intake backwards
    yButton.and(leftTrigger).whenActive(intake::backward, intake);
    // when the y button is, or the y button & left trigger are, inactive - stop.
    yButton.and(leftTrigger.whenInactive(intake::stop, intake)); 
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
