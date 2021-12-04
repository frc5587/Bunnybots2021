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
    public double getEncoderValue(EncoderValueType valueType) {
        switch(valueType) {
            case Position:
            return this.leader.getSelectedSensorPosition();
            case Velocity:
            return this.leader.getSelectedSensorVelocity();
            default:
            return 0;
        }
    }

    @Override
    public void setEncoderPosition(double position) {
        this.leader.setSelectedSensorPosition(position);
    }

    @Override
    public void setFPID() {
        this.leader.config_kP(ArmConstants.PID_SLOT, ArmConstants.ARM_PID.kP);
        this.leader.config_kI(ArmConstants.PID_SLOT, ArmConstants.ARM_PID.kI);
        this.leader.config_kD(ArmConstants.PID_SLOT, ArmConstants.ARM_PID.kD);
        this.leader.config_kF(ArmConstants.PID_SLOT, ArmConstants.ARM_PID.kF);
    }

    @Override
    public void setFeedForward(double ff) {
        this.leader.config_kF(ArmConstants.PID_SLOT, ff);
    }

    @Override
    public void configureMotors() {
        try {
            this.leader.configFactoryDefault();
        }
        catch(NullPointerException e) {
            System.out.println("NullPointerException " + e + " from arm leader configFactoryDefault");
        }
        this.leader.setNeutralMode(NeutralMode.Brake);
        this.leader.setInverted(ArmConstants.LEADER_INVERTED);

        for(WPI_TalonFX follower : this.followers) {
            try {
                follower.configFactoryDefault();
            }
            catch(NullPointerException e) {
                System.out.println("NullPointerException " + e + " from arm follower configFactoryDefault");
            }
            follower.setNeutralMode(NeutralMode.Brake);
            follower.setInverted(ArmConstants.FOLLOWERS_INVERTED);
        }
    }
}
