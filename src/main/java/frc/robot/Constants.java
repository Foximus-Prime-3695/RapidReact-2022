// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        // ------------------Subsystem Drive------------------
        public static final int LEFT_MASTER_ID = 1,
                        LEFT_SLAVE_ID = 2,
                        RIGHT_MASTER_ID = 3,
                        RIGHT_SLAVE_ID = 4;
        public static final boolean LEFT_MASTER_INVERT = false,
                        LEFT_SLAVE_INVERT = false,
                        RIGHT_MASTER_INVERT = true,
                        RIGHT_SLAVE_INVERT = true;
        public static final int CURRENT_LIMIT = 50;
        public static final double RAMP_RATE = 0.5D;
        public static final double AUTONOMOUS_DRIVE_INHIBITOR = 1D;
        public static final double AUTONOMOUS_ROTATION_INHIBITOR = 0.33D;
        // ------------------Subsystem Turret------------------
        public static final int YAW_MOTOR_ID = 5,
                        PITCH_MOTOR_ID = 15;
        // ------------------Subsystem Flywheel------------------
        public static final int FLYWHEEL_ID = 7;
        // ------------------Subsystem Intake------------------
        public static final int INTAKE_ADJUSTMENT_ID = 8;
        public static final int PRIMARY_INTAKE_ID = 9,
                        SECONDARY_INTAKE_ID = 10,
                        TERTIARY_INTAKE_ID = 11;
        // ------------------Subsystem Climb------------------
        public static final int CLIMB_ENGINE_ID = 12;
        public static final int LEFT_STAGE_ONE_ID = 0,
                        LEFT_STAGE_TWO_ID = 4,
                        RIGHT_STAGE_ONE_ID = 1,
                        RIGHT_STAGE_TWO_ID = 2;
        public static final double SPARK_MINIMUM_VOLTAGE = 8;
        public static final double TURRET_YAW_ABSOLUTE_MAX_OUTPUT = 0.65D;
        public static final int DEFAULT_TURRET_YAW_TICKS = 0;
        public static final int DEFAULT_TURRET_PITCH_TICKS = 0;
        public static final InvertType TURRET_PITCH_INVERT = InvertType.InvertMotorOutput;
        public static final InvertType TURRET_YAW_INVERT = InvertType.None;
        public static final int TURRET_YAW_AMP_LIMIT = 60;
        public static final double TURRET_YAW_ALLOWABLE_ERROR = 7.4D;
        public static final double TURRET_PITCH_ALLOWABLE_ERROR = 1000D;
        public static final double DRIVE_INHIBITOR = 0.85D;
        public static final double STEERING_INHIBITOR = 0.5D;
        public static final double DRIVE_ROTATIONS_PER_INCH = 0.472D;
        public static final double WHEEL_CIRCUMFERENCE = 18.653D;
        public static final double DRIVETRAIN_ALLOWABLE_ERROR = 6.0D;

        public static final int TURRET_CAMERA_PORT = 7778;
}
