package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants.DrivetrainConstants;
import org.frc5587.lib.subsystems.DrivetrainBase;

public class Drivetrain extends DrivetrainBase {
    public static int[] leftMotorIDs = new int[]{DrivetrainConstants.LEFT_LEADER, DrivetrainConstants.LEFT_FOLLOWER};
    public static int[] rightMotorIDs = new int[]{DrivetrainConstants.RIGHT_LEADER, DrivetrainConstants.RIGHT_FOLLOWER};
    // WPI_TalonFX[] leftFollowers = new WPI_TalonFX[leftMotorIDs.length-1];
    // WPI_TalonFX[] rightFollowers = new WPI_TalonFX[rightMotorIDs.length-1];

    public static DriveConstants constantsObj = new DriveConstants(
        DrivetrainConstants.TURN_FPID, 
        DrivetrainConstants.TURN_PID_FORWARD_THROTTLE, 
        DrivetrainConstants.TURN_PID_TOLERANCE_DEG,
        DrivetrainConstants.WHEEL_DIAMETER_METERS,
        DrivetrainConstants.HISTORY_LIMIT,
        DrivetrainConstants.INVERT_GYRO_DIRECTION,
        DrivetrainConstants.ENCODER_EDGES_PER_REV,
        DrivetrainConstants.GEARING
    );

    public Drivetrain() {
        super(constantsObj, leftMotorIDs, rightMotorIDs);
    }

    @Override
    public void configureMotors() {
        leftLeader.configFactoryDefault();
        rightLeader.configFactoryDefault();
        for(WPI_TalonFX follower : leftFollowers) {
            follower.configFactoryDefault();
            follower.setNeutralMode(NeutralMode.Brake);
        }
        for(WPI_TalonFX follower : rightFollowers) {
            follower.configFactoryDefault();
            follower.setNeutralMode(NeutralMode.Brake);
        }
        leftLeader.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        leftGroup.setInverted(DrivetrainConstants.LEFT_SIDE_INVERTED);
        rightGroup.setInverted(DrivetrainConstants.RIGHT_SIDE_INVERTED);
        // leftLeader.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        // rightLeader.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        // leftFollower.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        // rightFollower.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
    }

    @Override
    public void periodic() {
        super.periodic();
        System.out.println("X:  " + getPose() + "  Y:  " + getPose().getTranslation().getY() + "  R:  " + getHeading() + "  " + getPose().getRotation());
    }
}