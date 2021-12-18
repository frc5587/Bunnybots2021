package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.DrivetrainConstants;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperSimpleDrivetrain extends SubsystemBase {
    private final WPI_TalonFX leftLeader = new WPI_TalonFX(DrivetrainConstants.LEFT_LEADER);
    private final WPI_TalonFX leftFollower = new WPI_TalonFX(DrivetrainConstants.LEFT_FOLLOWER);
    private final WPI_TalonFX rightLeader = new WPI_TalonFX(DrivetrainConstants.RIGHT_LEADER);
    private final WPI_TalonFX rightFollower = new WPI_TalonFX(DrivetrainConstants.RIGHT_FOLLOWER);

    private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftLeader, leftFollower);
    private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightLeader, rightFollower);

    private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

    private final AHRS ahrs = new AHRS();
    private final DifferentialDriveOdometry odometry;

    public SuperSimpleDrivetrain() {
        odometry = new DifferentialDriveOdometry(ahrs.getRotation2d());

        resetEncoders();
        configureMotors();

    }

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

    @Override
    public void periodic() {
        super.periodic();
        odometry.update(ahrs.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());

        SmartDashboard.putNumber("X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Y", getPose().getTranslation().getY());
        SmartDashboard.putNumber("R", getHeading());

        SmartDashboard.putNumber("left v", getLeftVelocityMetersPerSecond());
        SmartDashboard.putNumber("right v", getRightVelocityMetersPerSecond());
    }

    public void resetEncoders() {
        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    private double getLeftPositionMeters() {
        return -applyEPR_Gearing_Distance(leftLeader.getSelectedSensorPosition());
    }

    private double getRightPositionMeters() {
        return applyEPR_Gearing_Distance(rightLeader.getSelectedSensorPosition());
    }

    private double getLeftVelocityMetersPerSecond() {
        return -applyEPR_Gearing_Distance(leftLeader.getSelectedSensorVelocity()) * 10;
    }

    private double getRightVelocityMetersPerSecond() {
        return applyEPR_Gearing_Distance(rightLeader.getSelectedSensorVelocity()) * 10;
    }

    private double applyEPR_Gearing_Distance(double rawEncoderTicks) {
        return (Math.PI * DrivetrainConstants.WHEEL_DIAMETER_METERS)
                * ((rawEncoderTicks / DrivetrainConstants.ENCODER_EDGES_PER_REV) / DrivetrainConstants.GEARING);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, ahrs.getRotation2d());
    }

    public void zeroOdometry() {
        resetOdometry(new Pose2d());
    }

    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(fwd, rot);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftGroup.setVoltage(-leftVolts);
        rightGroup.setVoltage(rightVolts); // TODO this might need to be negative or smth
        drive.feed();
    }

    public void zeroHeading() {
        ahrs.reset();
    }

    public double getHeading() {
        return ahrs.getRotation2d().getDegrees();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSecond(), getRightVelocityMetersPerSecond());
    }

    public void stop() {
        tankDriveVolts(0, 0);
    }

    public void crawl() {
        tankDriveVolts(DrivetrainConstants.CRAWL_THROTTLE, DrivetrainConstants.CRAWL_THROTTLE);
    }
}