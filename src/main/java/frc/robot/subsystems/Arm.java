package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.frc5587.lib.subsystems.PivotingArmBase;
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
    public static int[] motorIDs = new int[]{ArmConstants.ARM_LEADER, ArmConstants.ARM_FOLLOWER};
    
    public Arm() {
        super(constants, motorIDs);
    }

    @Override
    public void configureMotors() {
        leader.configFactoryDefault();
        leader.setNeutralMode(NeutralMode.Brake);
        leader.setInverted(ArmConstants.LEADER_INVERTED);
        for(WPI_TalonFX follower : followers) {
            follower.configFactoryDefault();
            follower.setNeutralMode(NeutralMode.Brake);
            follower.setInverted(ArmConstants.FOLLOWERS_INVERTED);
        }
    }
}
