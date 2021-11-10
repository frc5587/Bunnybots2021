package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Constants.DrivetrainConstants;
import org.frc5587.lib.subsystems.DrivetrainBase;


public class Drivetrain extends DrivetrainBase {
    private SpeedControllerGroup leftGroup, rightGroup;

    public Drivetrain() {
        super(new WPI_TalonFX(DrivetrainConstants.LEFT_LEADER), new WPI_TalonFX(DrivetrainConstants.RIGHT_LEADER), new WPI_TalonFX(DrivetrainConstants.LEFT_FOLLOWER), new WPI_TalonFX(DrivetrainConstants.RIGHT_FOLLOWER), DrivetrainConstants.TURN_FPID, DrivetrainConstants.TURN_PID_FORWARD_THROTTLE, DrivetrainConstants.INVERT_GYRO_DIRECTION);
    }

    @Override
    public void configureMotors() {
        leftGroup.setInverted(DrivetrainConstants.LEFT_SIDE_INVERTED);
        rightGroup.setInverted(DrivetrainConstants.RIGHT_SIDE_INVERTED);
    }
}