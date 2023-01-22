package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class PCH {
        public static final double NORMALIZED_SUPPLY_VOLTAGE = 0; //TODO: Figure out the value
    }

    public static final class Drivetrain {
        // public static final double TURN_kP = 0.00385;
        // public static final double TURN_kI = 0;
        // public static final double TURN_kD = 0.1;

        // public static final double DRIVE_GEAR_RATIO = 1;
        // public static final double TURN_GEAR_RATIO  = 1;

        // public static final double TOLERANCE = 5;
        // public static final double RATE_TOLERANCE = 0.5;

        // public static final boolean GYRO_REVERSED = true;
        

        // public static final double FRONT_LEFT_TALON_OFFSET_X = 0.5;
        // public static final double FRONT_LEFT_TALON_OFFSET_Y = 0.5;

        // public static final double FRONT_RIGHT_TALON_OFFSET_X = 0.5;
        // public static final double FRONT_RIGHT_TALON_OFFSET_Y = 0.5;

        // public static final double BACK_LEFT_TALON_OFFSET_X = 0.5;
        // public static final double BACK_LEFT_TALON_OFFSET_Y = 0.5;

        // public static final double BACK_RIGHT_TALON_OFFSET_X = 0.5;
        // public static final double BACK_RIGHT_TALON_OFFSET_Y = 0.5;  

        // public static final double FRONT_LEFT_OFFSET_ANGLE = 0;
        // public static final double FRONT_RIGHT_OFFSET_ANGLE = 0;
        // public static final double BACK_LEFT_OFFSET_ANGLE = 0;
        // public static final double BACK_RIGHT_OFFSET_ANGLE = 0;

        // public static final double driveKS = 0.71003;
        // public static final double driveKV = 2.2783;
        // public static final double driveKA = 0.25953;
        // public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(driveKS,driveKV,driveKA);
    
        // public static final double FWD_kP = 2;
        // public static final double STR_kP = 2;
        // public static final double THETA_kP = 3;
        // public static final double THETA_kD = .015;
        
        // public static final double MAX_ANGULAR_VELOCITY_RAD_TELEOP = .85*Math.PI;
        // public static final double MAX_TANGENTIAL_VELOCITY_TELEOP=3.5;

        // ----------FINALIZED----------
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

        // Dist between right and left wheels
        // Dist between front and back wheels
        public static final double DIST_MODULE_OFFSET = Units.inchesToMeters(22.5);

        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 21.4;
        
        // public static final double turnDefaultOffset = 0;
        // public static final double driveDefaultOffset = 0;

        public static final boolean FRONT_LEFT_D_IS_REVERSED = false;
        public static final boolean FRONT_RIGHT_D_IS_REVERSED = false;
        public static final boolean BACK_LEFT_D_IS_REVERSED = false;
        public static final boolean BACK_RIGHT_D_IS_REVERSED = false;

        public static final boolean FRONT_LEFT_T_IS_REVERSED = false;
        public static final boolean FRONT_RIGHT_T_IS_REVERSED = false;
        public static final boolean BACK_LEFT_T_IS_REVERSED = false;
        public static final boolean BACK_RIGHT_T_IS_REVERSED = false; 

        public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(313-180);
        public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(35);
        public static final double BACK_LEFT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(325);
        public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_RAD =  Math.toRadians(27);

        // ----------CONST----------
        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics (
            new Translation2d(DIST_MODULE_OFFSET / 2, DIST_MODULE_OFFSET / 2), 
            new Translation2d(DIST_MODULE_OFFSET / 2, -DIST_MODULE_OFFSET / 2), 
            new Translation2d(-DIST_MODULE_OFFSET / 2, DIST_MODULE_OFFSET / 2),
            new Translation2d(-DIST_MODULE_OFFSET / 2, -DIST_MODULE_OFFSET / 2)
        );
        
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final int GYRO_DELAY_MS = 1000;

        public static double kTicks2Radians(double ticks) {
            return Math.toRadians((ticks / 2048.0 / 21.4) * 360);
        }

        // ----------TUNED----------
        public static final boolean IS_FIELD_ORIENTED = false;
        // public static final double PID_CONST_TEST = 0.04;


        public static final double kPTurn = 0.001; // module
        public static final double kITurn = 0.000;
        public static final double kDTurn = 0.00015;
        // public static final double kPDrive = PID_CONST_TEST; // module
        // public static final double PID_CONTROLLER_X_P = PID_CONST_TEST; // not module idk
        // public static final double PID_CONTROLLER_Y_P = PID_CONST_TEST; // not module idk
        // public static final double PID_CONTROLLER_T_P = PID_CONST_TEST; // not module id
        
        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kPhysicalMaxSpeedMetersPerSecond = 0.5;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 0.5;
        
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 0.5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.0;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
            public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;

        
        
    }


    public static final class Shooter {
        public static double MAX_RPM = 4500;
        public static double TARGET_RPM = 3900;

        public static double P = 0;
        public static double I = 0;
        public static double D = 0;
        // public static double FF = 0.000205;
        public static double FF = 0.000205;

        public static int CURRENT_STALL_LIMIT = 30;
        public static int CURRENT_FREE_LIMIT = 30;
    }

    public static final class Climber {//min: -1.696402 max:
        public static double MAX_HEIGHT = 23;
        public static double MIN_HEIGHT = 0;
        public static double INCHES_TO_TICKS = 64/55/Math.PI*2048*8.5;
    }
}
