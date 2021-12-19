package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import org.frc5587.lib.subsystems.PivotingArmBase;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Arm extends PivotingArmBase {
    public static WPI_TalonFX[] motors = {new WPI_TalonFX(ArmConstants.ARM_LEADER), new WPI_TalonFX(ArmConstants.ARM_FOLLOWER)}; 

    public static FPIDConstants constants = new FPIDConstants(
        ArmConstants.ARM_SPEED_MULTIPLIER,
        ArmConstants.GEARING,
        ArmConstants.SOFT_LIMITS,
        ArmConstants.ZERO_OFFSET_TICKS,
        ArmConstants.ENCODER_CPR,
        ArmConstants.LIMIT_SWITCH,
        ArmConstants.LIMIT_SWITCH_INVERTED,
        ArmConstants.ARM_PID,
        ArmConstants.FEED_FORWARD,
        ArmConstants.CONSTRAINTS
    );

    public Arm() {
        this(motors);
    }
    
    public Arm(WPI_TalonFX[] motors) {
        super(constants, new SpeedControllerGroup(motors));
    }

    public DigitalInput getLimitSwitch() {
        return limitSwitch;
    }

    public boolean getLimitSwitchValue() {
        return (ArmConstants.LIMIT_SWITCH_INVERTED ? !limitSwitch.get() : limitSwitch.get());
    }
    
    @Override
    public double getEncoderPosition() {
        return motors[0].getSelectedSensorPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return motors[0].getSelectedSensorVelocity();
    }

    @Override
    public void setEncoderPosition(double position) {
        motors[0].setSelectedSensorPosition(position);
    }
    
    @Override
    public void configureMotors() {
        // try {
            for(WPI_TalonFX motor : motors) {
                motor.configFactoryDefault();
                motor.setNeutralMode(NeutralMode.Brake);
                motor.setInverted(ArmConstants.MOTORS_INVERTED);
                // motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            }
        // }
        // catch(NullPointerException e) {
        //     System.out.println("NullPointerException " + e + " from arm motor");
        // }
    }
    
    public void moveUp() {
        getController().setGoal(ArmConstants.HIGHER_SETPOINT);
    }
    
    public void moveDown() {
        getController().setGoal(ArmConstants.LOWER_SETPOINT);
    }
}
