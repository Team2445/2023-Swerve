// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Constants for Robot port numbers, PID values, etc. */
public class Constants {

    public static class OI {
        public static final int XBOX_CONTROLLER_PORT = 0;
    }

    // TODO: Change on actual robot
    public static class DrivetrainConstants {

        // Swerve drive measurements
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.25);
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(23.25);

        // Swerve drive motors
        // TODO: Find Module Offsets
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 0;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;
        public static final int FRONT_LEFT_MODULE_ENCODER = 0;
        public static final double FRONT_LEFT_MODULE_OFFSET = 0.0;

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
        public static final int FRONT_RIGHT_MODULE_ENCODER = 1;
        public static final double FRONT_RIGHT_MODULE_OFFSET = 0.0;

        public static final int REAR_LEFT_MODULE_DRIVE_MOTOR = 4;
        public static final int REAR_LEFT_MODULE_STEER_MOTOR = 5;
        public static final int REAR_LEFT_MODULE_ENCODER = 2;
        public static final double REAR_LEFT_MODULE_OFFSET = 0.0;

        public static final int REAR_RIGHT_MODULE_DRIVE_MOTOR = 6;
        public static final int REAR_RIGHT_MODULE_STEER_MOTOR = 7;
        public static final int REAR_RIGHT_MODULE_ENCODER = 3;
        public static final double REAR_RIGHT_MODULE_OFFSET = 0.0;

        public static final double MAX_VOLTAGE = 12.0;
        // TODO: Change these two later
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 3.0;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = -1.0;
    }
}
