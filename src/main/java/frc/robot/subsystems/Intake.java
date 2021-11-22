package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase {
    private final CANSparkMax rightIntake = new CANSparkMax(IntakeConstants.RIGHT_MOTOR, MotorType.kBrushless);
    private final CANSparkMax leftIntake = new CANSparkMax(IntakeConstants.LEFT_MOTOR, MotorType.kBrushless);

    public Intake() {
        super();

        configureSparkMax();
    }

    private void configureSparkMax() {
        rightIntake.restoreFactoryDefaults();
        leftIntake.restoreFactoryDefaults();

        rightIntake.setInverted(IntakeConstants.INVERTED);
        leftIntake.setInverted(IntakeConstants.INVERTED);

        rightIntake.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);
        leftIntake.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);

        rightIntake.setIdleMode(IdleMode.kBrake);
        leftIntake.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Moves intake forwards
     */
    public void forward() {
        rightIntake.set(IntakeConstants.THROTTLE);
        leftIntake.set(IntakeConstants.THROTTLE);
    }

    /**
     * Moves intake backwards
     */
    public void backward() {
        rightIntake.set(-IntakeConstants.THROTTLE);
        leftIntake.set(-IntakeConstants.THROTTLE);
    }

    /**
     * Stops intake motors
     */
    public void stop() {
        rightIntake.set(0);
        leftIntake.set(0);
    }
}
