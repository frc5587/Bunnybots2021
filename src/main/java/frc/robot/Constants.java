// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import org.frc5587.lib.pid.FPID;
import org.frc5587.lib.pid.PID;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import org.frc5587.lib.controllers.FFController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DrivetrainConstants {
        // motor ports 
        public static final int LEFT_LEADER = 10;
        public static final int LEFT_FOLLOWER = 11;
        public static final int RIGHT_LEADER = 15;
        public static final int RIGHT_FOLLOWER = 16;

        public static final boolean LEFT_SIDE_INVERTED = true;
        public static final boolean RIGHT_SIDE_INVERTED = true;
        
        // PID constants
        public static final FPID TURN_FPID = new FPID(0, 0.1, 0, 0.009);
        public static final double TURN_PID_TOLERANCE_DEG = 0.5;
        public static final double TURN_PID_FORWARD_THROTTLE = 0;
        public static final double TURN_PID_UPDATE_PERIOD_SEC = 0.010;
        public static final boolean INVERT_GYRO_DIRECTION = true;

        // motor current limit constants
        public static final int SMART_CURRENT_LIMIT = 35;
        public static final int HARD_CURRENT_LIMIT = 40;
        public static final double SMART_CURRENT_LIMIT_DELAY = 0.2; // seconds
        public static final StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT_CONFIGURATION = new StatorCurrentLimitConfiguration(
            true, 
            DrivetrainConstants.SMART_CURRENT_LIMIT, 
            DrivetrainConstants.HARD_CURRENT_LIMIT, 
            DrivetrainConstants.SMART_CURRENT_LIMIT_DELAY
        );
    
        public static final int TICKS_PER_REV = 8192;
        public static final double WHEEL_DIAMETER_METERS = 0.1524;
        public static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2;
        public static final int HISTORY_LIMIT = 32;
    }

    public static final class ArmConstants {
        // PID constants
        public static final double K_P = 3.5415;
        public static final double K_D = 0.10184;
        public static final PID ARM_PID = new PID(K_P, 0, K_D);
        public static final double VELOCITY_CONSTRAINT = 5;
        public static final double ACCELERATION_CONSTRAINT = 10;
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(
            VELOCITY_CONSTRAINT,
            ACCELERATION_CONSTRAINT
        );

        // Feedforward constants
        public static final double K_S = 0.5162;
        public static final double K_COS = 0.24835;
        public static final double K_V = 0.90019;
        public static final double K_A = 0.0083556;
        public static final FFController FEED_FORWARD = new FFController(K_S, K_COS, 0, K_V, K_A);

        // encoder calculation constants
        public static final double ARM_SPEED_MULTIPLIER = 0.3;
        public static final double GEARING = 55.125;
        public static final int ENCODER_CPR = 2048;
        public static final double LOWER_SETPOINT = Math.toRadians(5);
        public static final double HIGHER_SETPOINT = Math.toRadians(60);
        public static final int ZERO_OFFSET_TICKS = 313;
        public static final double[] SOFT_LIMITS = new double[]{0, Math.toRadians(65)};

        // ports
        public static final int LIMIT_SWITCH = 0;
        public static final int ARM_LEADER = 20;
        public static final int ARM_FOLLOWER = 21;

        public static final boolean MOTORS_INVERTED = false;
        public static final boolean LIMIT_SWITCH_INVERTED = false;
    }

    public static final class LEDConstants {
        public static final int PWM_PORT = 0;
        public static final int LED_LENGTH = 300; 
    }
    
    public static final class IntakeConstants {
        // motor ports
        public static final int RIGHT_MOTOR = 25;
        public static final int LEFT_MOTOR = 26;
        
        public static final boolean RIGHT_MOTOR_INVERTED = false;
        public static final boolean LEFT_MOTOR_INVERTED = true;
        // motor limits
        public static final int STALL_LIMIT = 20;
        public static final int FREE_LIMIT = 25;
        // motor speeds
        public static final double THROTTLE = 0.5;
    }

    public static class BunnyDumperConstants {
        public static final int[][] PISTON_PORTS = {{0, 1}};
    }
}