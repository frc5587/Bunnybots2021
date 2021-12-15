package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import org.frc5587.lib.controllers.FFController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.ArmConstants;

/**
* An arm subsystem that does not use a base class.
* This is built on {@link ProfiledPIDSubsystem} and uses Feedforward calculations from FFController
*/
public class FullArmSubsysTrapezoid extends ProfiledPIDSubsystem {
    private SpeedControllerGroup motorGroup;
    private WPI_TalonFX[] motors;
    private FFController ffController;
    protected ProfiledPIDController pidController;
    private DigitalInput limitSwitch;

    public FullArmSubsysTrapezoid() {
        super(
            new ProfiledPIDController(
                ArmConstants.ARM_PID.kP, 
                ArmConstants.ARM_PID.kI, 
                ArmConstants.ARM_PID.kD, 
                new TrapezoidProfile.Constraints(
                    ArmConstants.VELOCITY_CONSTRAINT,
                    ArmConstants.ACCELERATION_CONSTRAINT
                )
            )
        );
        
        this.enable();
        /**
        * create motors in an array so they can be accessed individually if needed (encoders, config, etc.)
        * 
        * then, put them in a SpeedControllerGroup so they can be controlled together
        */
        motors = new WPI_TalonFX[]{
            new WPI_TalonFX(ArmConstants.ARM_LEADER), 
            new WPI_TalonFX(ArmConstants.ARM_FOLLOWER)
        };
        motorGroup = new SpeedControllerGroup(motors);
        pidController = getController();
        // pidController.enableContinuousInput(0, 100);

        limitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH);
        /**
        * use FFPIDController to calculate Feedforward
        * these values come from Constants. we won't use kG because it's for elevator Feedforward.
        */
        ffController = new FFController(
            ArmConstants.K_S, 
            ArmConstants.K_COS, 
            0, 
            ArmConstants.K_V,
            ArmConstants.K_A
        );
        SmartDashboard.putBoolean("OUTPUT ON?", true);
    }

    /**
    * set every motor to break mode and invert it if needed. use integrated encoders.
    */
    public void configureMotors() {
        for(WPI_TalonFX motor : motors) {
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Brake);
            motor.setInverted(ArmConstants.MOTORS_INVERTED);
            motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        }
    }

    /**
    * @return the position of the arm in full rotations,
    * accounting for gearing and encoder counts per revolution.
    */
    public double getRotations() {
        try {
            return (motors[0].getSelectedSensorPosition() / ArmConstants.ENCODER_CPR / ArmConstants.GEARING);
        }
        catch(NullPointerException e) {
            System.out.println(e + " Could not get Encoder");
            return 0;
        }
    }

    /**
     * @return the position of the arm in degrees,
     * accounting for gearing and encoder counts per revolution.
     */
    public double getAngleDegrees() {
        return getRotations() * 360;
    }

    /**
    * this should be used for most calculations (including Feedforward and getMeasurement)
    * @return the position of the arm in radians.
    */
    public double getAngleRadians() {
        return Math.toRadians(getAngleDegrees());
    }

    public double getVelocityRadians() {
        return (motors[0].getSelectedSensorVelocity() / 2048 / 55.125) * 2 * Math.PI;
    }

    /**
    * @param value the percent output to set the motors to (a double -1 to 1)
    */
    public void set(double value) {
        motorGroup.set(value);
    }

    public void setAngleDegrees(double angle) {
        pidController.setGoal(Math.toRadians(angle));
    }

    /**
    * @param voltage the voltage to set the motors to
    */
    public void setVoltage(double voltage) {
        motorGroup.setVoltage(voltage);
    }

    /**
    * sets the leader's encoder to 0
    */
    public void resetEncoders() {
        motors[0].setSelectedSensorPosition(313);
    }

    /**
    * stops all motors
    */
    public void stop() {
        motorGroup.set(0);
    }

    public boolean getLimitSwitchValue() {
        return (ArmConstants.LIMIT_SWITCH_INVERTED ? !limitSwitch.get() : limitSwitch.get());
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // double ff = ffController.calculateArm(getMeasurement(), getVelocityRadians());
        // SmartDashboard.putNumber("FF", ff);
        SmartDashboard.putNumber("OUTPUT USED", output);
        SmartDashboard.putNumber("SETPOINT USED", setpoint.position);
        SmartDashboard.putNumber("GOAL USED", pidController.getGoal().position);
        SmartDashboard.putBoolean("AT SETPOINT", pidController.atGoal());

        if(getLimitSwitchValue() && output < 0) {
            setVoltage(0);
        }

        else if(!getLimitSwitchValue() && getMeasurement() > Math.toRadians(65) && output > 0) {
            setVoltage(0);
        }

        else {
            setVoltage(output);
        }
    }

    @Override
    public void periodic() {
        // double goal = Math.toRadians(40);
        // pidController.setGoal(goal);
        TrapezoidProfile.State goalState = pidController.getGoal();
        double ff = ffController.calculateArm(goalState.position, goalState.velocity);
        // double ff = ffController.calculateArm(goalState.position);
        double output = pidController.calculate(getMeasurement(), goalState.position);
        SmartDashboard.putNumber("Angle in Radians", getMeasurement());
        SmartDashboard.putNumber("Angle in Degrees", getAngleDegrees());
        SmartDashboard.putNumber("FeedForward", ff);
        SmartDashboard.putNumber("Output calculated", output);
        SmartDashboard.putNumber("Output passed", output + ff);
        SmartDashboard.putNumber("Goal", goalState.position);
        System.out.println(SmartDashboard.getBoolean("OUTPUT ON?", true));
        Shuffleboard.update();
        if(SmartDashboard.getBoolean("OUTPUT ON?", true)) {
            useOutput(ff + output, pidController.getGoal());
        }
        else {
            useOutput(0, new TrapezoidProfile.State());
        }
        // useOutput(ff + output, pidController.getGoal());
    }

    @Override
    public double getMeasurement() {
        return getAngleRadians();
    }
}