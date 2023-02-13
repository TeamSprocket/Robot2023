package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Constants {
    public static final class PCH {
        public static final double NORMALIZED_SUPPLY_VOLTAGE = 0; //TODO: Figure out the value
        public static final int MAX_PSI = 120;
        public static final int MIN_PSI = 80;
    }

    public static final class Drivetrain {
        public static final double TURN_kP = 0.00385;
        public static final double TURN_kI = 0;
        public static final double TURN_kD = 0.1;

        public static final double TOLERANCE = 5;
        public static final double RATE_TOLERANCE = 0.5;

        public static final boolean GYRO_REVERSED = true;   
        
        public static final int PIDIDX = 0;
        public static final int ENCODER_CPR = 2048;
        public static final double WHEEL_DIAMETER_METERS = 0.152;
        public static final double ENCODER_DISTANCE_PER_PULSE = (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.654;
        public static final double kvVoltSecondsPerMeter = 2.76;
        public static final double kaVoltSecondsSquaredPerMeter = 0.593;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 1.82;
        public static final double kTrackwidthMeters = 0.7112;
        public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class Shooter {
        public static double MAX_RPM = 2000;
        public static double BOTTOM_TARGET_RPM = 1230; //closer to intake (front of bot)
        public static double BOTTOM_SECOND_TARGET_RPM = 1410;
        public static double TOP_TARGET_RPM = 750; //closer to battery (back of bot)

        public static double P = 0.015;
        public static double I = 0;
        public static double D = 0.01;
        // public static double FF = 0.000205;
        public static double FF = 0.5;

        public static final double TOLERANCE_RPM = 35;
        public static final double NORMAL_RATIO = 2;

        public static int CURRENT_STALL_LIMIT = 30;
        public static int CURRENT_FREE_LIMIT = 30;
    }

    public static final class Climber {
        public static double MAX_HEIGHT = -0.09275;
        public static double MIN_HEIGHT = -22.658;
        public static double INCHES_TO_TICKS = 64/55/Math.PI*2048*8.5;
    }
}