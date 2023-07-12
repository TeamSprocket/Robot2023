package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    // ----------Don't Touch----------
    public static boolean isEnabled = false;
    public static boolean isPreciseDrive = false;
    // -------------------------------

    public static double kTeleopMultiplier = 1.1;
    public static final int GYRO_DELAY_MS = 1000;

    public static final class Measurements {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

        public static final double DIST_MODULE_OFFSET = 0.572;

        public static final double kDriveMotorGearRatio = 6.75;
        public static final double kTurningMotorGearRatio = 21.35;
    }

    public static final class Drivetrain {
        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics (
            new Translation2d(Measurements.DIST_MODULE_OFFSET / 2, Measurements.DIST_MODULE_OFFSET / 2),
            new Translation2d(-Measurements.DIST_MODULE_OFFSET / 2, Measurements.DIST_MODULE_OFFSET / 2), 
            new Translation2d(Measurements.DIST_MODULE_OFFSET / 2, -Measurements.DIST_MODULE_OFFSET / 2),
            new Translation2d(-Measurements.DIST_MODULE_OFFSET / 2, -Measurements.DIST_MODULE_OFFSET / 2)
        );

        public static final boolean FRONT_LEFT_D_IS_REVERSED = false;
        public static final boolean FRONT_RIGHT_D_IS_REVERSED = false;
        public static final boolean BACK_LEFT_D_IS_REVERSED = false;
        public static final boolean BACK_RIGHT_D_IS_REVERSED = false;

        public static final boolean FRONT_LEFT_T_IS_REVERSED = false;
        public static final boolean FRONT_RIGHT_T_IS_REVERSED = false;
        public static final boolean BACK_LEFT_T_IS_REVERSED = false;
        public static final boolean BACK_RIGHT_T_IS_REVERSED = false;

        public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(0.0);
        public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(0.0);
        public static final double BACK_LEFT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(0.0);
        public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_RAD =  Math.toRadians(0.0);

        public static final double kPTurn = 0.6; //0.6
        public static final double kITurn = 0.0000;
        public static final double kDTurn = 0.0015; 
        
        public static final double kMaxSpeedDrive = 0.1; 
        public static final double kMaxAccelDrive = 0.5;
        public static final double kMaxSpeedTurn = 0.2; 
        public static final double kMaxAccelTurn = 0.5;

        public static final double CURRENT_LIMIT_TURN = 100;
        public static final double CURRENT_LIMIT_DRIVE = 100;

        public static final double PRECISE_DRIVE_SPEED_PERCENT = 0.25;
    }

    public static final class Auton { 
        public static final boolean FACING_DRIVERS = true;
    }


}









