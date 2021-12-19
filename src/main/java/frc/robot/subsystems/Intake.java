package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final CANSparkMax rightIntake = new CANSparkMax(IntakeConstants.RIGHT_MOTOR, MotorType.kBrushless);
    private final CANSparkMax leftIntake = new CANSparkMax(IntakeConstants.LEFT_MOTOR, MotorType.kBrushless);

    private final CANEncoder rightEncoder = rightIntake.getEncoder();
    private final CANEncoder leftEncoder = leftIntake.getEncoder();

    private final SpeedControllerGroup intakeMotors = new SpeedControllerGroup(leftIntake, rightIntake);

    private double lastSet = 0;

    public Intake() {
        configureMotors();
    }

    // @Override
    public void configureMotors() {
        rightIntake.restoreFactoryDefaults();
        leftIntake.restoreFactoryDefaults();

        rightIntake.setInverted(IntakeConstants.RIGHT_MOTOR_INVERTED);
        leftIntake.setInverted(IntakeConstants.LEFT_MOTOR_INVERTED);

        rightIntake.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);
        leftIntake.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);

        rightIntake.setIdleMode(IdleMode.kBrake);
        leftIntake.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Moves intake forwards
     */
    public void forward() {
        intakeMotors.set(IntakeConstants.THROTTLE_FORWARD);
        lastSet = IntakeConstants.THROTTLE_FORWARD;
    }
    
    /**
     * Moves intake backwards
     */
    public void backward() {
        intakeMotors.set(-IntakeConstants.THROTTLE_REVERSE);
        lastSet = -IntakeConstants.THROTTLE_REVERSE;
    }
    
    /**
     * Stops intake
     */
    public void stop() {
        intakeMotors.set(IntakeConstants.HOLD);
        lastSet = 0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("has crate", hasCrate()? 1:0);

        SmartDashboard.putNumber("left v", leftVelocity());
        SmartDashboard.putNumber("right v", rightVelocity());
    }

    private double leftVelocity() {
        return leftEncoder.getVelocity();
    }

    private double rightVelocity() {
        return rightEncoder.getVelocity();
    }

    public boolean hasCrate() {
        if (lastSet > 0) {
            // System.out.println(rightVelocity() > IntakeConstants.RIGHT_VELOCITY_THRESHOLD && leftVelocity() < IntakeConstants.LEFT_VELOCITY_THRESHOLD);
            return rightVelocity() > IntakeConstants.RIGHT_VELOCITY_THRESHOLD && leftVelocity() < IntakeConstants.LEFT_VELOCITY_THRESHOLD;
        } else {
            // System.out.println("Intake is not spinning forward, cannot detect if crate is grabbed");
            return false; // should this throw an error?
        }
    }
}
