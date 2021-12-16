// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandJoystick;
import org.frc5587.lib.control.DeadbandXboxController;
import org.frc5587.lib.advanced.AddressableLEDController;

import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.BunnyDumper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
    private final Intake intake = new Intake();
    private final BunnyDumper bunnyDumper = new BunnyDumper();
    private final Arm arm = new Arm();
    // Commands
    private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, joy::getY, () -> -joy.getXCurveDampened());
    // Others
    private final AddressableLEDController ledController = new AddressableLEDController(
        Constants.LEDConstants.PWM_PORT,
        Constants.LEDConstants.LED_LENGTH
    );
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
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Y Button for Intake controls
        JoystickButton yButton = new JoystickButton(xboxController, XboxController.Button.kY.value);
        // B Button for Bunny Dumper controls
        JoystickButton bButton = new JoystickButton(xboxController, XboxController.Button.kB.value);
        // Left Trigger for Intake & Bunny Dumper controls
        Trigger leftTrigger = new Trigger(() -> xboxController.getTrigger(Hand.kLeft));
        // DPad Up for higher arm setpoint
        POVButton dpadUp = new POVButton(xboxController, 0);
        // DPad Down for lower arm setpoint
        POVButton dpadDown = new POVButton(xboxController, 180);
        // right trigger for manual arm control
        Trigger rightTrigger = new Trigger(() -> xboxController.getTrigger(Hand.kRight));
        // arm limit switch
        Trigger armLimitSwitch = new Trigger(() -> arm.getLimitSwitchValue());

        /**
         * Bunny Dumper
         */
        
        // when b button and left trigger are pressed together, extend the pistons. when
        // the two buttons are released, retract the pistons
        bButton.and(leftTrigger).whenActive(bunnyDumper::extend, bunnyDumper)
                .whenInactive(bunnyDumper::retract, bunnyDumper);
                
        /*
         * Intake
         */

        // when y button is active, move intake forwards | when the y button is inactive
        // - stop.
        yButton.and(leftTrigger.negate()).whenActive(intake::forward, intake).whenInactive(intake::stop, intake);
        // when y button & left trigger are active, move intake backwards | when the y
        // button & left trigger are inactive - stop.
        yButton.and(leftTrigger).whenActive(intake::backward, intake).whenInactive(intake::stop, intake);

        /* 
         * Arm
         */ 
        armLimitSwitch.whenActive(arm::resetEncoders);

        dpadUp.whenPressed(
            () -> arm.getController().setGoal(Constants.ArmConstants.HIGHER_SETPOINT), arm
        );

        dpadDown.whenPressed(
            () -> arm.getController().setGoal(Constants.ArmConstants.LOWER_SETPOINT), arm
        );

        // while X is held
        // rightTrigger.whileActiveContinuous(
        //     () -> {
        //         // make sure useOutput() is not being used by periodic()
        //         SmartDashboard.putBoolean("OUTPUT ON?", false);
        //         // set the arm to the output of the right xbox controller stick
        //         arm.set(
        //             xboxController.getY(Hand.kRight) * Constants.ArmConstants.ARM_SPEED_MULTIPLIER
        //         );
        //     },
        // arm)
        // //when X is released, turn output back on.
        // .whenInactive(() -> SmartDashboard.putBoolean("OUTPUT ON?", true), arm);
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
