package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.DrivetrainConstants;
import org.frc5587.lib.subsystems.DrivetrainBase;

public class Drivetrain extends DrivetrainBase {
    public static int[] leftMotorIDs = new int[]{DrivetrainConstants.LEFT_LEADER, DrivetrainConstants.LEFT_FOLLOWER};
    public static int[] rightMotorIDs = new int[]{DrivetrainConstants.RIGHT_LEADER, DrivetrainConstants.RIGHT_FOLLOWER};

    public static DriveConstants constantsObj = new DriveConstants(
        DrivetrainConstants.TURN_FPID, 
        DrivetrainConstants.TURN_PID_FORWARD_THROTTLE, 
        DrivetrainConstants.TURN_PID_TOLERANCE_DEG,
        DrivetrainConstants.WHEEL_DIAMETER_METERS,
        DrivetrainConstants.HISTORY_LIMIT,
        DrivetrainConstants.INVERT_GYRO_DIRECTION
    );

    public Drivetrain() {
        super(constantsObj, leftMotorIDs, rightMotorIDs);
    }

    @Override
    public void configureMotors() {
        leftLeader.configFactoryDefault();
        rightLeader.configFactoryDefault();
        leftFollower.configFactoryDefault();
        rightFollower.configFactoryDefault();
        leftLeader.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);
        leftGroup.setInverted(DrivetrainConstants.LEFT_SIDE_INVERTED);
        rightGroup.setInverted(DrivetrainConstants.RIGHT_SIDE_INVERTED);
        // leftLeader.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        // rightLeader.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        // leftFollower.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        // rightFollower.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
    }
}