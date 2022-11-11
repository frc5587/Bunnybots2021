package frc.robot.subsystems;

import org.frc5587.lib.subsystems.DrivetrainBase;
import frc.robot.Constants.DrivetrainConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends DrivetrainBase {
    private static final CANSparkMax leftMotor = new CANSparkMax(DrivetrainConstants.LEFT_MOTOR, MotorType.kBrushless);
    private static final CANSparkMax rightMotor = new CANSparkMax(DrivetrainConstants.RIGHT_MOTOR, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

    private static DriveConstants driveConstants = new DriveConstants(
        DrivetrainConstants.WHEEL_DIAMETER_METERS,
        DrivetrainConstants.HISTORY_LIMIT, 
        DrivetrainConstants.INVERT_GYRO_DIRECTION,
        DrivetrainConstants.ENCODER_EDGES_PER_REV, 
        DrivetrainConstants.GEARING,
        DrivetrainConstants.TRACK_WIDTH
    );

    public Drivetrain() {
        super(new MotorControllerGroup(leftMotor), new MotorControllerGroup(rightMotor), driveConstants);
        zeroOdometry();

        SmartDashboard.putBoolean("BrakeMode", true);
    }

    @Override
    public void configureMotors() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setInverted(DrivetrainConstants.LEFT_SIDE_INVERTED);
        rightMotor.setInverted(DrivetrainConstants.RIGHT_SIDE_INVERTED);

        leftMotor.setSmartCurrentLimit(DrivetrainConstants.SMART_CURRENT_LIMIT, DrivetrainConstants.HARD_CURRENT_LIMIT);
        rightMotor.setSmartCurrentLimit(DrivetrainConstants.SMART_CURRENT_LIMIT, DrivetrainConstants.HARD_CURRENT_LIMIT);

        resetEncoders();
    }

    @Override
    protected double getLeftPositionTicks() {
        return leftEncoder.getPosition() * (DrivetrainConstants.LEFT_SIDE_INVERTED ? -1:1);
    }

    @Override
    protected double getRightPositionTicks() {
        return rightEncoder.getPosition() * (DrivetrainConstants.RIGHT_SIDE_INVERTED ? -1:1);
    }

    @Override
    protected double getLeftVelocityTicksPerSecond() {
        return (leftEncoder.getVelocity() * (DrivetrainConstants.LEFT_SIDE_INVERTED ? -1:1) / DrivetrainConstants.VELOCITY_COEFFICIENT);
    }

    @Override
    protected double getRightVelocityTicksPerSecond() {
        return (rightEncoder.getVelocity() * (DrivetrainConstants.RIGHT_SIDE_INVERTED ? -1:1) / DrivetrainConstants.VELOCITY_COEFFICIENT);
    }

    @Override
    protected void resetEncoders() {
        // leftEncoder.setPosition(0);
        // rightEncoder.setPosition(0);
    }

    @Override
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        super.tankDriveVolts(-leftVolts, -rightVolts);
    }

    @Override
    public void periodic() {
        if(SmartDashboard.getBoolean("BrakeMode", true) == true) {
            leftMotor.setIdleMode(IdleMode.kBrake);
            rightMotor.setIdleMode(IdleMode.kBrake);
        }
        else {
            leftMotor.setIdleMode(IdleMode.kCoast);
            rightMotor.setIdleMode(IdleMode.kCoast);
        }
    }
}