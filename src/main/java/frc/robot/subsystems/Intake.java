package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
// import org.frc5587.lib.subsystems.SimpleMotorBase;

import edu.wpi.first.wpilibj.SpeedController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
    private final CANSparkMax rightIntake = new CANSparkMax(IntakeConstants.RIGHT_MOTOR, MotorType.kBrushless);
    private final CANSparkMax leftIntake = new CANSparkMax(IntakeConstants.LEFT_MOTOR, MotorType.kBrushless);

    private final CANEncoder rightEncoder = rightIntake.getEncoder();
    private final CANEncoder leftEncoder = leftIntake.getEncoder();

    private final SpeedControllerGroup intakeMotors = new SpeedControllerGroup(leftIntake, rightIntake);

    private final PowerDistributionPanel pdp = new PowerDistributionPanel();

    private double lastVelocity, nowVelocity, lastSet = 0;
    private boolean useVelocityDetection;

    public Intake() {
        this(true);
    }

    public Intake(boolean velocityDetection) {
        useVelocityDetection = velocityDetection;
        // super(new SpeedController[]{rightIntake, leftIntake}, IntakeConstants.THROTTLE);
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
        intakeMotors.set(0);
        lastSet = 0;
    }

    @Override
    public void periodic() {
        lastVelocity = nowVelocity;
        nowVelocity = getAverageVelocity();

        SmartDashboard.putNumber("Current", getAverageCurrent());
        SmartDashboard.putNumber("Velocity", getAverageVelocity());
        SmartDashboard.putNumber("Acceleration", getAbsoluteAverageAcceleration());
        SmartDashboard.putNumber("ratio", nowVelocity / getAverageCurrent());

        SmartDashboard.putNumber("has crate", hasCrate()? 1:0);
        SmartDashboard.putNumber("is crate ejected", isCrateEjected()? 1:0);
    }



    private double getAverageCurrent() {
        // System.out.println("PDP: " + pdp.getCurrent(IntakeConstants.PDP_SLOT_1) + "    "+ pdp.getTemperature() + "   " + pdp.getTotalCurrent() + "   " + pdp.getVoltage());
        return (pdp.getCurrent(IntakeConstants.PDP_SLOT_1) + pdp.getCurrent(IntakeConstants.PDP_SLOT_2)) / 2;
    }

    private double leftVelocity() {
        return leftEncoder.getVelocity();
    }

    private double rightVelocity() {
        return rightEncoder.getVelocity();
    }

    private double getAverageVelocity() {
        return (rightVelocity() + leftVelocity()) / 2;
    }

    private double getAbsoluteAverageAcceleration() {
        return Math.abs((nowVelocity - lastVelocity) / 0.02);
    }

    public boolean hasCrate() {
        if (useVelocityDetection) {
            if (lastSet > 0) {
                return rightVelocity() < IntakeConstants.RIGHT_VELOCITY_THRESHOLD && leftVelocity() > IntakeConstants.LEFT_VELOCITY_THRESHOLD;
            } else {
                System.out.println("Intake is not spinning forward, cannot detect if crate is grabbed");
                return false; // should this throw an error?
            }
        } else {
            return nowVelocity / getAverageCurrent() < IntakeConstants.STALL_VELOCITY_CURRENT_THRESHOLD && getAbsoluteAverageAcceleration() < IntakeConstants.STALL_ACCELERATION_THRESHOLD;
        }
    }

    public boolean isCrateEjected() {
        if (lastSet < 0) {
            return nowVelocity / getAverageCurrent() > IntakeConstants.EJECTING_VELOCITY_CURRENT_THRESHOLD && getAbsoluteAverageAcceleration() > IntakeConstants.STALL_ACCELERATION_THRESHOLD;
        } else {
            System.out.println("Intake is not reversing, cannot detect if crate is ejected");
            return false; // should this throw an error?
        }
    }
}
