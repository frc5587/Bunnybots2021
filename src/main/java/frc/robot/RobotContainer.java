// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandJoystick;
import org.frc5587.lib.control.DeadbandXboxController;

import java.util.ArrayList;

import org.frc5587.lib.advanced.AddressableLEDController;
import org.frc5587.lib.auto.AutoPath;
import org.frc5587.lib.auto.RamseteCommandWrapper;
import org.frc5587.lib.advanced.RainbowLEDPattern;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.EjectCrate;
import frc.robot.commands.IntakeCrate;
import frc.robot.subsystems.BunnyDumper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SuperSimpleDrivetrain;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Intake;

import frc.robot.Constants.LEDConstants;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    // private final Drivetrain drivetrain = new Drivetrain();
    private final AddressableLEDController ledController = new AddressableLEDController(LEDConstants.PWM_PORT, LEDConstants.LED_LENGTH, new RainbowLEDPattern(LEDConstants.LED_SPEED, LEDConstants.LED_LENGTH / 2, LEDConstants.LED_LENGTH, 255));
    private final SuperSimpleDrivetrain ssDrivetrain = new SuperSimpleDrivetrain();
    private final BunnyDumper bunnyDumper = new BunnyDumper();
    private final Intake intake = new Intake();
    // Commands
    private final ArcadeDrive arcadeDrive = new ArcadeDrive(ssDrivetrain, joy::getY, () -> -joy.getXCurveDampened());
    private final IntakeCrate intakeCrate = new IntakeCrate(intake);
    private final EjectCrate ejectCrate = new EjectCrate(intake);
    // Auto paths
    private final RamseteCommandWrapper getRightBox = new RamseteCommandWrapper(ssDrivetrain, new AutoPath("get right box"), AutoConstants.RAMSETE_CONSTANTS); 
    private final RamseteCommandWrapper dropOffRightBox = new RamseteCommandWrapper(ssDrivetrain, new AutoPath("drop off right box"), AutoConstants.RAMSETE_CONSTANTS); 
    private final RamseteCommandWrapper backupAndCenter = new RamseteCommandWrapper(ssDrivetrain, new AutoPath("backup and center"), AutoConstants.RAMSETE_CONSTANTS); 
    private final RamseteCommandWrapper getRightBox2 = new RamseteCommandWrapper(ssDrivetrain, new AutoPath("get right box 2"), AutoConstants.RAMSETE_CONSTANTS); 
    private final RamseteCommandWrapper dropOffRightBox2 = new RamseteCommandWrapper(ssDrivetrain, new AutoPath("drop off right box 2"), AutoConstants.RAMSETE_CONSTANTS); 
    private final RamseteCommandWrapper backupAndCenter2 = new RamseteCommandWrapper(ssDrivetrain, new AutoPath("backup and center 2"), AutoConstants.RAMSETE_CONSTANTS); 
    // Testing auto paths
    private final RamseteCommandWrapper sPath = new RamseteCommandWrapper(ssDrivetrain, new AutoPath("s path"), AutoConstants.RAMSETE_CONSTANTS).resetOdometryOnStart(); 
    private final RamseteCommandWrapper y2Meters = new RamseteCommandWrapper(ssDrivetrain, new Pose2d(0, 0, new Rotation2d(0)), new ArrayList<Translation2d>(), new Pose2d(0, 2, new Rotation2d(0)), AutoConstants.RAMSETE_CONSTANTS); 
    private final RamseteCommandWrapper x2Meters = new RamseteCommandWrapper(ssDrivetrain, new Pose2d(0, 0, new Rotation2d(0)), new ArrayList<Translation2d>(), new Pose2d(2, 0, new Rotation2d(0)), AutoConstants.RAMSETE_CONSTANTS); 

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // make ssDrivetrain use arcadeDrive to drive
        ssDrivetrain.setDefaultCommand(arcadeDrive);
        // Configure the button bindings
        configureButtonBindings();
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
         * Bunny Dumper
         */

        // when b button and left trigger are pressed together, extend the pistons. when
        // the two buttons are released, retract the pistons
        bButton.and(leftTrigger).whenActive(bunnyDumper::extend, bunnyDumper).whenInactive(bunnyDumper::retract,
                bunnyDumper);

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
        return y2Meters.zeroOdometryOnStart(); //TODO this should move forward 2 meters

        // return new SequentialCommandGroup(new ParallelCommandGroup(getRightBox.resetOdometryOnStart(), intakeCrate), dropOffRightBox, ejectCrate, backupAndCenter);

        // return new SequentialCommandGroup(new ParallelCommandGroup(getRightBox.resetOdometryOnStart(), intakeCrate), dropOffRightBox, ejectCrate, new ParallelCommandGroup(getRightBox2, intakeCrate), ejectCrate, backupAndCenter2);

        // return new SequentialCommandGroup(getRightBox, dropOffRightBox, backupAndCenter);
    }
}
