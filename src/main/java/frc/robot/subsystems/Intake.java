package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import org.frc5587.lib.subsystems.SimpleMotorBase;

import edu.wpi.first.wpilibj.SpeedController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SimpleMotorBase {
    private static final CANSparkMax rightIntake = new CANSparkMax(IntakeConstants.RIGHT_MOTOR, MotorType.kBrushless);
    private static final CANSparkMax leftIntake = new CANSparkMax(IntakeConstants.LEFT_MOTOR, MotorType.kBrushless);

    public Intake() {
        super(new SpeedController[]{rightIntake, leftIntake}, IntakeConstants.THROTTLE);
    }

    @Override
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
}
