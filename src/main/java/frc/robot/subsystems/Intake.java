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


public class Intake extends SubsystemBase {
    private final CANSparkMax rightIntake = new CANSparkMax(IntakeConstants.RIGHT_MOTOR, MotorType.kBrushless);
    private final CANSparkMax leftIntake = new CANSparkMax(IntakeConstants.LEFT_MOTOR, MotorType.kBrushless);

    private final CANEncoder rightEncoder = rightIntake.getEncoder();
    private final CANEncoder leftEncoder = leftIntake.getEncoder();

    private final SpeedControllerGroup intakeMotors = new SpeedControllerGroup(leftIntake, rightIntake);

    private final PowerDistributionPanel pdp = new PowerDistributionPanel();

    private double lastVelocity, nowVelocity, lastSet = 0;

    public Intake() {
        super();

        configureSparkMax();
    }

    private void configureSparkMax() {
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
        intakeMotors.set(IntakeConstants.THROTTLE);
        lastSet = IntakeConstants.THROTTLE;
    }
    
    /**
     * Moves intake backwards
     */
    public void backward() {
        intakeMotors.set(-IntakeConstants.THROTTLE);
        lastSet = -IntakeConstants.THROTTLE;
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

        SmartDashboard.putNumber("Stalled", isStalled()? 1:0);
    }



    private double getAverageCurrent() {
        // System.out.println("PDP: " + pdp.getCurrent(IntakeConstants.PDP_SLOT_1) + "    "+ pdp.getTemperature() + "   " + pdp.getTotalCurrent() + "   " + pdp.getVoltage());
        return (pdp.getCurrent(IntakeConstants.PDP_SLOT_1) + pdp.getCurrent(IntakeConstants.PDP_SLOT_2)) / 2;
    }

    private double getAverageVelocity() {
        return (rightEncoder.getVelocity() + leftEncoder.getVelocity()) / 2;
    }

    private double getAbsoluteAverageAcceleration() {
        return Math.abs((nowVelocity - lastVelocity) / 0.02);
    }

    public boolean isStalled() {
        return nowVelocity / getAverageCurrent() < IntakeConstants.STALL_VELOCITY_CURRENT_THRESHOLD && getAbsoluteAverageAcceleration() < IntakeConstants.STALL_ACCELERATION_THRESHOLD;
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
