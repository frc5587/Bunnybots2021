package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.frc5587.lib.subsystems.PivotingArmBase;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ArmConstants;

public class Arm extends PivotingArmBase {
    public static ArmsConstants constants = new ArmsConstants(
        ArmConstants.ARM_SPEED_MULTIPLIER, 
        ArmConstants.ARM_LENGTH_INCHES, 
        ArmConstants.LIMIT_SWITCH,
        ArmConstants.PID_SLOT,
        ArmConstants.ARM_PID,
        ArmConstants.FEED_FORWARD
    );
    public static WPI_TalonFX[] motors = new WPI_TalonFX[]{new WPI_TalonFX(ArmConstants.ARM_LEADER), new WPI_TalonFX(ArmConstants.ARM_FOLLOWER)};
    
    public Arm() {
        super(constants, motors);
    }

    @Override
    public double getEncoderValue(EncoderValueType valueType) {
        switch(valueType) {
            case kPosition:
            return motors[0].getSelectedSensorPosition();
            case kVelocity:
            return motors[0].getSelectedSensorVelocity();
            default:
            return 0;
        }
    }

    @Override
    public void setEncoderPosition(double position) {
        motors[0].setSelectedSensorPosition(position);
    }

    @Override
    public void configureMotors() {
        motors[0].configFactoryDefault();
        motors[0].setNeutralMode(NeutralMode.Brake);
        leader.setInverted(ArmConstants.LEADER_INVERTED);
        for(WPI_TalonFX follower : (WPI_TalonFX[]) followers) {
            follower.configFactoryDefault();
            follower.setNeutralMode(NeutralMode.Brake);
            follower.setInverted(ArmConstants.FOLLOWERS_INVERTED);
        }
    }
}
