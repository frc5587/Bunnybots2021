package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import org.frc5587.lib.controllers.FFController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

/**
* An arm subsystem that does not use a base class.
* This is built on {@link PIDSubsystem} and uses Feedforward calculations from FFController
*/
public class FullArmSubsys extends PIDSubsystem {
    private SpeedControllerGroup motorGroup;
    private WPI_TalonFX[] motors;
    private FFController ffController;
    private PIDController pidController;
    private DigitalInput limitSwitch;

    public FullArmSubsys() {
        super(new PIDController(ArmConstants.ARM_PID.kP, ArmConstants.ARM_PID.kI, ArmConstants.ARM_PID.kD));
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
        return (motors[0].getSelectedSensorPosition() / ArmConstants.ENCODER_CPR / ArmConstants.GEARING);
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
        pidController.setSetpoint(angle);
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
        motors[0].setSelectedSensorPosition(1254);
        // System.out.println("*********" + motors[0].getSelectedSensorPosition());
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
    public void useOutput(double output, double setpoint) {
        System.out.println("OUTPUT: " + output);
        set(output);
    }

    @Override
    public void periodic() {
        double setpoint = Math.toRadians(40);
        double ff = ffController.calculateArm(getMeasurement());
        double output = getController().calculate(getMeasurement(), setpoint);
        SmartDashboard.putNumber("Angle in Radians", getMeasurement());
        SmartDashboard.putNumber("Angle in Degrees", getAngleDegrees());
        SmartDashboard.putNumber("FeedForward", ff);
        SmartDashboard.putNumber("Setpoint", getSetpoint());
        SmartDashboard.putNumber("Output calculated", output);
        SmartDashboard.putNumber("Output used", output + ff);
        Shuffleboard.update();
        useOutput(ff + output, getSetpoint());
    }

    @Override
    public double getMeasurement() {
        return getAngleRadians();
    }
}