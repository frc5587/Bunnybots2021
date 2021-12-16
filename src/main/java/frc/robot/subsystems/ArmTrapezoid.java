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
* An arm subsystem with Trapezoidal PID and Feedforward control.
* This is built on {@link ProfiledPIDSubsystem} and uses Feedforward calculations from FFController
*/
public class ArmTrapezoid extends ProfiledPIDSubsystem {
    private SpeedControllerGroup motorGroup;
    private WPI_TalonFX[] motors;
    private FFController ffController;
    protected ProfiledPIDController pidController;
    private DigitalInput limitSwitch;

    public ArmTrapezoid() {
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
        * then, put them in a SpeedControllerGroup so they can be controlled together
        */
        motors = new WPI_TalonFX[]{
            new WPI_TalonFX(ArmConstants.ARM_LEADER), 
            new WPI_TalonFX(ArmConstants.ARM_FOLLOWER)
        };
        motorGroup = new SpeedControllerGroup(motors);
        pidController = getController();

        limitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH);
        /**
        * Use FFController to calculate Feedforward
        * These values come from Constants. We won't use kG because it's for elevator Feedforward.
        */
        ffController = new FFController(
            ArmConstants.K_S, 
            ArmConstants.K_COS, 
            0, 
            ArmConstants.K_V,
            ArmConstants.K_A
        );
        /** turn ON periodic's use of useOutput */
        SmartDashboard.putBoolean("OUTPUT ON?", true);
    }

    /**
    * Set every motor to brake mode and invert it if needed. Use integrated encoders.
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
    * @return the position of the arm in radians.
    */
    public double getAngleRadians() {
        return Math.toRadians(getAngleDegrees());
    }

    /**
    * @return the velocity of the arm in radians per second
    */
    public double getVelocityRadians() {
        return (motors[0].getSelectedSensorVelocity() / 2048 / 55.125) * 2 * Math.PI;
    }

    /**
    * Sets the arm motors to a percent output
    * @param value the percent output to set the motors to (a double -1 to 1)
    */
    public void set(double value) {
        motorGroup.set(value);
    }

    /**
    * Sets the arm's goal position to an angle in radians
    * @param angleRadians the desired arm angle in radians
    */
    public void setAngleRadians(double angleRadians) {
        pidController.setGoal(angleRadians);
    }

    /**
    * Sets the arm's goal position to an angle in degrees
    * @param angleDegrees the desired arm angle in degrees
    */
    public void setAngleDegrees(double angleDegrees) {
        pidController.setGoal(Math.toRadians(angleDegrees));
    }

    /**
    * @param voltage the voltage to set the motors to
    */
    public void setVoltage(double voltage) {
        motorGroup.setVoltage(voltage);
    }

    /**
    * sets the leader's encoder to the zero offset defined in Constants
    */
    public void resetEncoders() {
        motors[0].setSelectedSensorPosition(ArmConstants.ZERO_OFFSET_TICKS);
    }

    /**
    * stops all motors
    */
    public void stop() {
        motorGroup.set(0);
    }

    /**
     * @return the limit switch's state, inverted if necessary.
     */
    public boolean getLimitSwitchValue() {
        return (ArmConstants.LIMIT_SWITCH_INVERTED ? !limitSwitch.get() : limitSwitch.get());
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
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
        TrapezoidProfile.State goalState = pidController.getGoal();
        /** calculate Feedforward and PID with position and velocity from the pidController */
        double ff = ffController.calculateArm(goalState.position, goalState.velocity);
        double output = pidController.calculate(getMeasurement(), goalState.position);
        /** print various values to SmartDashboard */
        SmartDashboard.putNumber("Angle in Radians", getMeasurement());
        SmartDashboard.putNumber("Angle in Degrees", getAngleDegrees());
        SmartDashboard.putNumber("FeedForward", ff);
        SmartDashboard.putNumber("Output calculated", output);
        SmartDashboard.putNumber("Output passed", output + ff);
        SmartDashboard.putNumber("Goal", goalState.position);
        Shuffleboard.update();
        /** if the driver has set output on, useOutput. */
        if(SmartDashboard.getBoolean("OUTPUT ON?", true)) {
            /** output should be feedforward + calculated PID. */
            useOutput(ff + output, pidController.getGoal());
        }
        /** otherwise, set output to 0 */
        else {
            useOutput(0, new TrapezoidProfile.State());
        }
    }

    @Override
    public double getMeasurement() {
        return getAngleRadians();
    }
}