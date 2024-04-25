package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    
    /** Constants for Swerve Drive. All arrays in order of index. */
    public static class DriveConstants {
        // Index order: Front Left, Back Left, Front Right, Back Right

        public static final int[] DRIVE_PORTS = {
            0, 1, 2, 3
        };

        public static final int[] TURNING_PORTS = {
            4, 5, 6, 7
        };

        public static final int[] CANCODER_PORTS = {
            8, 9, 10, 11
        };

        public static final boolean[] DRIVE_REVERSED = {
            true, true, true, true
        };

        public static final boolean[] TURNING_REVERSED = {
            true, true, true, true
        };

        public static final boolean[] CANCODER_REVERSED = {
            false, false, false,
        };

        public static final double[] ENCODER_OFFSETS = {
            (19.42 - 0.1) / 360.0,
            (-35.94 - 0.2 + 0.3) / 360.0,
            (-89.03 - 1.5 + 0.2) / 360.0,
            (64.16 - 0.8) / 360.0
        };

        public static final double kTrackWidth = Units.inchesToMeters(24.0);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24.0);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2));

        public static final int kCurrentLimit = 45;
        public static final int kSmartCurrentLimit = 40;

        public static class ModuleConstants {
            public static final double kWheelDiameterMeters = Units.inchesToMeters(
                4);
            public static final double kDriveMotorGearRatio = 1.0 / 6.75;
            public static final double kTurningMotorGearRatio = 7.0 / 150.0;
            public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
            public static final double kTurningEncoderRot2Deg = kTurningMotorGearRatio * 360.0;
            public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
            public static final double kTurningEncoderRPM2DegPerSec = kTurningEncoderRot2Deg / 60;
            
            public static final double kPDrive = 0.0;
            public static final double kIDrive = 0.0;
            public static final double kDDrive = 0.0;

            public static final double kPTurning = 0.3;
            public static final double kITurning = 0.0;
            public static final double kDTurning = 0.0;
            public static final double MAX_TURN_ERROR_DEGREES = 0.5;
        }
    }
}
