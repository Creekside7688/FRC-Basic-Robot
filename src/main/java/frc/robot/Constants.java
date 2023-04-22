// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Constants {
    public static class OperatorConstants {
        public static final int CONTROLLER_PORT = 0;
    }

    public static class DrivetrainConstants {
        public static final double TRACK_WIDTH_METRES = 0.61;

        public static final double KS_VOLTS = 0.22;
        public static final double KV_VOLT_SECONDS_PER_METRE = 1.98;
        public static final double KA_VOLT_SECONDS_SQUARED_PER_METRE = 0.2;
        public static final double KP_DRIVE = 8.5;

        public static final DifferentialDriveKinematics DRIVE_TRAIN_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH_METRES);

        public static final double RAMSETE_B = 2;

        public static final double RAMSETE_ZETA = 0.7;

        public static final int TLF_MOTOR = 1;
        public static final int TLB_MOTOR = 2;
        public static final int BLF_MOTOR = 3;
        public static final int BLB_MOTOR = 4;

        public static final int TRF_MOTOR = 5;
        public static final int TRB_MOTOR = 6;
        public static final int BRF_MOTOR = 7;
        public static final int BRB_MOTOR = 8;

        public static final int[] LEFT_ENCODER = new int[] { 0, 1 };
        public static final int[] RIGHT_ENCODER = new int[] { 2, 3 };

        public static final double WHEEL_PERIMETER_CM = 15.24 * Math.PI;
        public static final double DISTANCE_PER_PULSE = WHEEL_PERIMETER_CM / 2048.0;

        public static class Auto {
            public static final double MAX_SPEED_METRES_PER_SECOND = 3;
            public static final double MAX_ACCELERATION_METRES_PER_SECOND_SQUARED = 1;
        }
    }
}
