package frc.robot.subsystems;

import java.util.Hashtable;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants.DrivetrainConstants;
import org.frc5587.lib.subsystems.DrivetrainBase;

public class Drivetrain extends DrivetrainBase {
    public static Hashtable<String, Object> constants = new Hashtable<String, Object>() {{
        put("TURN_FPID", DrivetrainConstants.TURN_FPID);
        put("TURN_PID_FORWARD_THROTTLE", DrivetrainConstants.TURN_PID_FORWARD_THROTTLE);
        put("INVERT_GYRO_DIRECTION", DrivetrainConstants.INVERT_GYRO_DIRECTION);
        put("WHEEL_DIAMETER_METERS", DrivetrainConstants.WHEEL_DIAMETER_METERS);
        put("HISTORY_LIMIT", DrivetrainConstants.HISTORY_LIMIT);
        put("TURN_PID_TOLERANCE_DEG", DrivetrainConstants.TURN_PID_TOLERANCE_DEG);
        put("LEFT_SIDE_INVERTED", DrivetrainConstants.LEFT_SIDE_INVERTED);
        put("RIGHT_SIDE_INVERTED", DrivetrainConstants.RIGHT_SIDE_INVERTED);
    }};
    public Drivetrain() {
        super(new WPI_TalonFX(DrivetrainConstants.LEFT_LEADER), new WPI_TalonFX(DrivetrainConstants.RIGHT_LEADER), new WPI_TalonFX(DrivetrainConstants.LEFT_FOLLOWER), new WPI_TalonFX(DrivetrainConstants.RIGHT_FOLLOWER), constants);
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