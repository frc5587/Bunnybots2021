package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import org.frc5587.lib.controllers.FFPIDController;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class FullArmSubsys extends PIDSubsystem {
    private SpeedControllerGroup motorGroup;
    private WPI_TalonFX[] motors;
    private FFPIDController ffpidController;

    public FullArmSubsys() {
        super(new PIDController(ArmConstants.ARM_PID.kP, ArmConstants.ARM_PID.kI, ArmConstants.ARM_PID.kD));
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

        /**
        * use FFPIDController to calculate Feedforward
        * these values come from Constants. we won't use kG because it's for elevator Feedforward.
        */
        ffpidController = new FFPIDController(
            ArmConstants.K_S, 
            ArmConstants.K_COS, 
            0, 
            ArmConstants.K_V,
            ArmConstants.K_A
        );
    }

    public void configureMotors() {
        /**
        * set every motor to break mode and invert it if needed.
        */
        for(WPI_TalonFX motor : motors) {
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Brake);
            motor.setInverted(ArmConstants.MOTORS_INVERTED);
        }
    }

    /**
    * @return the position of the arm in full rotations
    * this accounts for gearing and encoder counts per revolution.
    */
    public double getRotations() {
        return motors[0].getSelectedSensorPosition() / ArmConstants.ENCODER_CPR / ArmConstants.GEARING;
    }

    /**
     * @return the position of the arm in degrees
     * this accounts for gearing and encoder counts per revolution.
     */
    public double getAngleDegrees() {
        return getRotations() * 360;
    }

    /**
    * @return the position of the arm in radians
    * this should be used for most calculations (including Feedforward and getMeasurement)
    */
    public double getAngleRadians() {
        return Math.toRadians(getAngleDegrees());
    }

    /**
    * @param value the percent output to set the motors to (a double -1 to 1)
    */
    public void set(double value) {
        motorGroup.set(value);
    }

    /**
    * @param the voltage to set the motors to
    */
    public void setVoltage(double voltage) {
        motorGroup.setVoltage(voltage);
    }

    /**
    * sets the leader's encoder to 0
    */
    public void resetEncoders() {
        motors[0].setSelectedSensorPosition(0);
    }

    /**
    * stops all motors
    */
    public void stop() {
        motorGroup.set(0);
    }

    @Override
    public void useOutput(double output, double setpoint) {
        set(output);
        System.out.println(output);
    }

    @Override
    public void periodic() {
        double setpoint = 10;
        double output = getController().calculate(getMeasurement(), setpoint);
        /** DEBUG */
        // useOutput(output, setpoint);
        System.out.println("" + getMeasurement() + "      " + getRotations() + "         ");
        System.out.println("FF is" + ffpidController.calculateArm(getMeasurement()));
        useOutput(output + ffpidController.calculateArm(getMeasurement()), setpoint);
    }

    @Override
    public double getMeasurement() {
        return getAngleRadians();
    }
}