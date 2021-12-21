package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.frc5587.lib.subsystems.DrivetrainBase;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends DrivetrainBase {
    private final WPI_TalonFX leftLeader;
    private final WPI_TalonFX leftFollower;
    private final WPI_TalonFX rightLeader;
    private final WPI_TalonFX rightFollower;

    private static DriveConstants driveConstants = new DriveConstants(DrivetrainConstants.WHEEL_DIAMETER_METERS,
            DrivetrainConstants.HISTORY_LIMIT, DrivetrainConstants.INVERT_GYRO_DIRECTION,
            DrivetrainConstants.ENCODER_EDGES_PER_REV, DrivetrainConstants.GEARING,
            DrivetrainConstants.FLIP_LEFT_ENCODERS, DrivetrainConstants.FLIP_RIGHT_ENCODERS);

    /**
     * Default constructor that initializes the motors to be passed in.
     */
    public Drivetrain() {
        this(new WPI_TalonFX(DrivetrainConstants.LEFT_LEADER), new WPI_TalonFX(DrivetrainConstants.LEFT_FOLLOWER),
                new WPI_TalonFX(DrivetrainConstants.RIGHT_LEADER), new WPI_TalonFX(DrivetrainConstants.RIGHT_FOLLOWER));
    }

    /**
     * Takes the motors and passes them into the {@link DrivetrainBase} and the initializes the motors objects (used in `configureMotors()`)
     * 
     * @param leftLeader talonFX left leader
     * @param leftFollower talonFX left follower
     * @param rightLeader talonFX right leader
     * @param rightFollower talonFX right follower
     */
    public Drivetrain(WPI_TalonFX leftLeader, WPI_TalonFX leftFollower, WPI_TalonFX rightLeader,
            WPI_TalonFX rightFollower) {
        super(new SpeedControllerGroup(leftLeader, leftFollower), new SpeedControllerGroup(rightLeader, rightFollower),
                driveConstants);

        this.leftLeader = leftLeader;
        this.leftFollower = leftFollower;
        this.rightLeader = rightLeader;
        this.rightFollower = rightFollower;
    }

    /**
     * Configures the motors. This is automatically called in the constructor of {@link DrivetrainBase}.
     */
    @Override
    public void configureMotors() {
        leftLeader.configFactoryDefault();
        leftFollower.configFactoryDefault();
        rightLeader.configFactoryDefault();
        rightFollower.configFactoryDefault();

        leftLeader.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);

        leftLeader.setInverted(DrivetrainConstants.LEFT_SIDE_INVERTED);
        leftFollower.setInverted(DrivetrainConstants.LEFT_SIDE_INVERTED);
        rightLeader.setInverted(DrivetrainConstants.RIGHT_SIDE_INVERTED);
        rightFollower.setInverted(DrivetrainConstants.RIGHT_SIDE_INVERTED);

        leftLeader.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        rightLeader.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        leftFollower.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        rightFollower.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);

    }

    // * ENCODER METHODS
    
    @Override
    protected double getRightPositionTicks() {
        return rightLeader.getSelectedSensorPosition();
    }

    @Override
    protected double getLeftPositionTicks() {
        return leftLeader.getSelectedSensorPosition();
    }

    @Override
    protected double getRightVelocityTicksPerSecond() {
        return rightLeader.getSelectedSensorVelocity();
    }

    @Override
    protected double getLeftVelocityTicksPerSecond() {
        return leftLeader.getSelectedSensorVelocity();
    }

    @Override
    protected void resetEncoders() {
        rightLeader.setSelectedSensorPosition(0);
        leftLeader.setSelectedSensorPosition(0);

    }
}