// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandJoystick;
import org.frc5587.lib.control.DeadbandXboxController;

import java.util.function.DoubleSupplier;

import org.frc5587.lib.advanced.AddressableLEDController;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmMovementThrottle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FullArmSubsys;
import frc.robot.subsystems.FullArmSubsysTrapezoid;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID;
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
    private final DeadbandXboxController xb = new DeadbandXboxController(1);
    // Subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    private final FullArmSubsysTrapezoid arm = new FullArmSubsysTrapezoid();
    // Commands
    private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, joy::getY, () -> -joy.getXCurveDampened());
    // private final ArmMovementThrottle armMovementThrottle = new ArmMovementThrottle(arm, () -> xb.getY(Hand.kRight));
    // Others
    private final AddressableLEDController ledController = new AddressableLEDController(LEDConstants.PWM_PORT,
            LEDConstants.LED_LENGTH);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // make drivetrain use arcadeDrive to drive
        drivetrain.setDefaultCommand(arcadeDrive);
        // make the arm use the xbox joystick to move
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
    private void configureButtonBindings() {
        POVButton dpadUp = new POVButton(xb, 0);
        POVButton dpadDown = new POVButton(xb, 180);
        JoystickButton bButton = new JoystickButton(xb, XboxController.Button.kX.value);
        Trigger armLimitSwitch = new Trigger(() -> arm.getLimitSwitchValue());

        armLimitSwitch.whenActive(arm::resetEncoders);
        dpadUp.whenPressed(
            () -> {
                arm.setGoal(1);
                arm.enable();
                },
            arm
        );

        dpadDown.whenPressed(
            () -> {
                arm.setGoal(0.069);
                arm.enable();
                },
            arm
        );

        bButton.whenPressed(() -> {arm.stop(); arm.disable();}, arm);
        // dpadUp.whileActiveContinuous(arm::moveByFixedSpeed, arm).whenInactive(arm::stop, arm);
        // dpadDown.and(armLimitSwitch).whileActiveContinuous(arm::moveFixedReversed, arm).whenInactive(arm::stop, arm);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
