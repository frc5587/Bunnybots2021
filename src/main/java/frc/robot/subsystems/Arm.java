package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import org.frc5587.lib.subsystems.PivotingArmBase;

public class Arm extends PivotingArmBase {
    public static WPI_TalonFX[] motors = new WPI_TalonFX[]{new WPI_TalonFX(ArmConstants.ARM_LEADER), new WPI_TalonFX(ArmConstants.ARM_FOLLOWER)};
    private WPI_TalonFX leader = motors[0];
    private WPI_TalonFX[] followers = new WPI_TalonFX[]{motors[1]};

    public static ArmsConstants constants = new ArmsConstants(
        ArmConstants.ARM_SPEED_MULTIPLIER, 
        ArmConstants.ARM_LENGTH_INCHES, 
        ArmConstants.LIMIT_SWITCH,
        ArmConstants.PID_SLOT,
        ArmConstants.ARM_PID,
        ArmConstants.FEED_FORWARD
    );
    
    public Arm() {
        super(constants, motors);
    }

    @Override
    public void configureMotors() {
        try {
            this.leader.configFactoryDefault();
            this.leader.setNeutralMode(NeutralMode.Brake);
            this.leader.setInverted(ArmConstants.LEADER_INVERTED);
            resetEncoders();
        }
        catch(NullPointerException e) {
            System.out.println("NullPointerException " + e + " from arm leader motor");
        }
        try {
            for(WPI_TalonFX follower : this.followers) {
                follower.configFactoryDefault();
                follower.setNeutralMode(NeutralMode.Brake);
                follower.setInverted(ArmConstants.FOLLOWERS_INVERTED);
            }
        }
        catch(NullPointerException e) {
            System.out.println("NullPointerException " + e + " from arm follower motor");
        }
    }
}
