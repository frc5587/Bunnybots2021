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

    /**
     * Configures the motors, this includes inversions, current limits, and idle modes.
     */
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
     * Moves intake inwards
     */
    public void in() {
        intakeMotors.set(IntakeConstants.THROTTLE_FORWARD);
        lastSet = IntakeConstants.THROTTLE_FORWARD;
    }
    
    /**
     * Moves intake outwards
     */
    public void out() {
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

    private double leftVelocity() {
        return leftEncoder.getVelocity();
    }

    private double rightVelocity() {
        return rightEncoder.getVelocity();
    }

    /**
     * Detects crate if right motors is stalled and left motor is spinning under very slight load. This is just a result of the elastic on the left side of the intake.
     * 
     * @return whether crate is detected
     */
    public boolean hasCrate() {
        if (lastSet > 0) {
            return rightVelocity() > IntakeConstants.RIGHT_VELOCITY_THRESHOLD && leftVelocity() < IntakeConstants.LEFT_VELOCITY_THRESHOLD;
        } else {
            return false; 
        }
    }
}
