package frc.robot;

public final class Constants {

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class IntakeConstants {
        public static final int kMotorId = 1;
        public static final int kCurrentLimit = 40;
        public static final double kIntakeSpeed = -0.35 * 12.0;
        public static final double kReverseSpeed = 0.5 * 12.0;
    }

    public static final class ShooterConstants {
        public static final int kLeftMotorId = 4;
        public static final int kRightMotorId = 5;
        public static final int kCurrentLimit = 40;
        public static final double kShootSpeed = 0.65 * 12.0;
    }

    public static final class TransportConstants {
        public static final int kLeftMotorId = 2;
        public static final int kRightMotorId = 6;
        public static final int kCurrentLimit = 60;
        public static final double kTransportSpeed = 0.35 * 12.0;
    }

    public static final class VisionConstants {
        public static final String kLimelightName = "emperor";
        public static final double kCloseEnoughDistanceMin = 1.2; // Meters
        public static final double kCloseEnoughDistanceMax = 1.5; // Meters
        
        // Alignment PID Constants
        public static final double kAlignmentP = 0.1;
        public static final double kAlignmentI = 0.0;
        public static final double kAlignmentD = 0.01;
        public static final double kAlignmentTolerance = 1.0; // Degrees
    }

    public static final class DriveConstants {
        public static final double kDeadband = 0.15 ;
    }
}
