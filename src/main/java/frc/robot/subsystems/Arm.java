package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import org.frc5587.lib.subsystems.PivotingArmBase;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Arm extends PivotingArmBase {
    public static WPI_TalonFX[] motors = new WPI_TalonFX[]{new WPI_TalonFX(ArmConstants.ARM_LEADER), new WPI_TalonFX(ArmConstants.ARM_FOLLOWER)};
    public static SpeedControllerGroup motorGroup = new SpeedControllerGroup(motors);
    private WPI_TalonFX leader = motors[0];
    private DigitalInput limitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH);

    public static FPIDConstants constants = new FPIDConstants(
        ArmConstants.ARM_SPEED_MULTIPLIER,
        ArmConstants.GEARING,
        ArmConstants.ENCODER_CPR,
        ArmConstants.ARM_PID,
        ArmConstants.FEED_FORWARD
    );
    
    public Arm() {
        super(constants, motorGroup);
    }

    public DigitalInput getLimitSwitch() {
        return limitSwitch;
    }

    public boolean getLimitSwitchValue() {
        return (ArmConstants.LIMIT_SWITCH_INVERTED ? !limitSwitch.get() : limitSwitch.get());
    }
    
    @Override
    public double getEncoderPosition() {
        return -this.leader.getSelectedSensorPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return -this.leader.getSelectedSensorVelocity();
    }

    @Override
    public void setEncoderPosition(double position) {
        this.leader.setSelectedSensorPosition(position);
    }
    
    @Override
    public void configureMotors() {
        try {
            for(WPI_TalonFX motor : motors) {
                motor.configFactoryDefault();
                motor.setNeutralMode(NeutralMode.Brake);
                motor.setInverted(ArmConstants.MOTORS_INVERTED);
            }
        }
        catch(NullPointerException e) {
            System.out.println("NullPointerException " + e + " from arm motor");
        }
    }
}
