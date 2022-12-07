// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;

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
        public static final double DDR_FWD = 0.2;
        public static final double DDR_TURN = 0.2;
        // motor ports 
        public static final int LEFT_MOTOR = 10;
        public static final int RIGHT_MOTOR = 15;

        public static final boolean LEFT_SIDE_INVERTED = true;
        public static final boolean RIGHT_SIDE_INVERTED = true;

        public static final boolean FLIP_LEFT_ENCODERS = true;
        public static final boolean FLIP_RIGHT_ENCODERS = false;

        public static final boolean INVERT_GYRO_DIRECTION = true;

        // motor current limit constants
        public static final int SMART_CURRENT_LIMIT = 35;
        public static final int HARD_CURRENT_LIMIT = 40;
    
        public static final int TICKS_PER_REV = 8192;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6);
        public static final int HISTORY_LIMIT = 32;

        public static final int ENCODER_EDGES_PER_REV = 2048;
        public static final double GEARING = (54./20.) * (50./12.);

        public static final double VELOCITY_COEFFICIENT = 10; // CTRE measures velocity in units per 100ms, so this makes it units per 1s
        public static final double TRACK_WIDTH = 0.683;
    }
}