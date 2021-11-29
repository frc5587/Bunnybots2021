package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase {
    private final CANSparkMax rightIntake = new CANSparkMax(IntakeConstants.RIGHT_MOTOR, MotorType.kBrushless);
    private final CANSparkMax leftIntake = new CANSparkMax(IntakeConstants.LEFT_MOTOR, MotorType.kBrushless);

    private final SpeedControllerGroup intakeMotors = new SpeedControllerGroup(rightIntake, leftIntake);

    public Intake() {
        super();

        configureSparkMax();
    }

    private void configureSparkMax() {
        intakeMotors.restoreFactoryDefaults();

        rightIntake.setInverted(IntakeConstants.RIGHT_MOTOR_INVERTED);
        leftIntake.setInverted(IntakeConstants.LEFT_MOTOR_INVERTED);

        intakeMotors.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);

        intakeMotors.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Moves intake forwards
     */
    public void forward() {
        intakeMotors.set(IntakeConstants.THROTTLE);
    }

    /**
     * Moves intake backwards
     */
    public void backward() {
        intakeMotors.set(-IntakeConstants.THROTTLE);
    }

    /**
     * Stops intake
     */
    public void stop() {
        intakeMotors.set(0);
    }
}
