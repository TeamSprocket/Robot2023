package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static boolean isTeleop = false;
    public static double kTeleopMultiplier = 1.1;

    //TODO Edit PID values, tune speeds
    public static final class Elevator {
    
        //check sprocket radius in inches
        public static double kSprocketRadius = 2.938;
        public static double kElevatorGearRatio = 9;

        //height between base & starting config
        public static double offset = 33.638;

        public static double kP = 0.175;
        public static double kI = 0;
        public static double kD = 0.00675;
        public static double FF = 0;
        
        //Max/min heights in inches;
        public static double MAX_HEIGHT = -13.12711;
        public static double MIN_HEIGHT = 34.45866374;
        public static double TOTAL_HEIGHT = MAX_HEIGHT - MIN_HEIGHT; 

        public static int CURRENT_STALL_LIMIT = 30;
        public static int CURRENT_FREE_LIMIT = 30;
    }

    public static final class Arm{
        
        public static double kArmGearRatio = 31.5;

        public static double angleConversionFactor = 360.0 / kArmGearRatio;

        public static double maxAngle = -80;
        public static double groundAngle = -22;
        public static double startingAngle = -6;
        
        public static double kP = 0.05;
        public static double kI = 0;
        public static double kD = 0.0003;
        public static double FF = 0;

    }

    public static final class Wrist {
        
        public static double kWristGearRatio = 2.75;

        public static double angleConversionFactor = 360.0 / (kWristGearRatio * 10);
        
        public static double P = 0.0175;
        public static double I = 0;
        public static double D = 0.0007;
        public static double FF = 0;

        public static double maxAngle;
        public static double minAngle;
        public static double angleRange;

    }

    public static final class Claw {
        public static double revSetpoint = 400;

        public static double kPShooter = 0.03;
        public static double kIShooter = 0;
        public static double kDShooter = 0.00;
        
    }

    public static final class PCH {
        public static double MIN_PSI = 80;
        public static double MAX_PSI = 100;
    }

    public static final class Drivetrain {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

        public static final double DIST_MODULE_OFFSET = 0.572;

        public static final double kDriveMotorGearRatio = 6.75;
        public static final double kTurningMotorGearRatio = 21.35;

        public static final boolean BACK_RIGHT_D_IS_REVERSED = false;
        public static final boolean FRONT_RIGHT_D_IS_REVERSED = false;
        public static final boolean BACK_LEFT_D_IS_REVERSED = false;
        public static final boolean FRONT_LEFT_D_IS_REVERSED = false;

        public static final boolean BACK_RIGHT_T_IS_REVERSED = false;
        public static final boolean FRONT_RIGHT_T_IS_REVERSED = false;
        public static final boolean BACK_LEFT_T_IS_REVERSED = false;
        public static final boolean FRONT_LEFT_T_IS_REVERSED = false; 

        // public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(56.78);
        // public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(117.86);
        // public static final double BACK_LEFT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(40.78);
        // public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_RAD =  Math.toRadians(117.86);

        public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_RAD =  Math.toRadians(303.2);
        public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(305.5);
        public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(44.6);
        public static final double BACK_LEFT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(77.5);

        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics (
            new Translation2d(DIST_MODULE_OFFSET / 2, DIST_MODULE_OFFSET / 2),
            new Translation2d(-DIST_MODULE_OFFSET / 2, DIST_MODULE_OFFSET / 2), 
            new Translation2d(DIST_MODULE_OFFSET / 2, -DIST_MODULE_OFFSET / 2),
            new Translation2d(-DIST_MODULE_OFFSET / 2, -DIST_MODULE_OFFSET / 2)
        );
        
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final int GYRO_DELAY_MS = 1000;

        public static final boolean IS_FIELD_ORIENTED = true;
        public static final boolean TURN_MANUAL = false;

        public static final double kPTurn = 0.6; //0.6
        public static final double kITurn = 0.0000;
        public static final double kDTurn = 0.0015; 
        
        public static final double kMaxSpeedMetersPerSecond = 0.2; //0.2
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1;
        
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 0.4;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.25;

        public static final double kPhysicalMaxSpeedMetersPerSecond = kMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;   
        
        public static double kTicks2Radians(double ticks) {
            return Math.toRadians((ticks / 2048.0 / 21.4) * 360);
        }

        public static boolean SWERVE_IS_SLOW = false;

		public static boolean JOYSTICK_DRIVING_ENABLED = true;

        public static final double kLimelightP = 0.0075;
        public static final double kLimelightI = 0.0;
        public static final double kLimelightD = 0.00015;

        public static final double CURRENT_LIMIT_TURN = 100;

        public static final double PRECISE_DRIVE_SPEED_PERCENT = 0.25;

    }

    public static final class Auton { 
        public static final boolean FACING_DRIVERS = true;

        public static final double kPTurn = 0.3; //0.2
        public static final double kITurn = 0;
        public static final double kDTurn = 0.0015; //0.001

        public static final double kPBalance = 0.003; //0.08
        public static final double kIBalance = 0;
        public static final double kDBalance = 0.001;
        public static final double BALANCE_END_TIME_THRESHOLD = 0.25; // 0.5
        public static final double BALANCE_END_ANGLE_THRESHOLD = 5; 
        

        public static final double SPEED_ON_RAMP = 0.0365; // 0.019
        public static final double CHARGING_STATION_WAIT_OFFSET = 1.5;

        public static final double[][] ROUTINE = {{0.0, 0.0, 0.0, 0.0, 2.356194490192345, -2.356194490192345, 0.7853981633974483, -0.7853981633974483}, {0.0, 0.0, 0.0, 0.0, 2.356194490192345, -2.356194490192345, 0.7853981633974483, -0.7853981633974483}, {0.01632753567810988, 0.01632753567810988, 0.01632753567810988, 0.01632753567810988, 0.5992400782190582, 0.5992400782190582, 0.5992400782190582, 0.5992400782190582}, {0.045247694500382775, 0.045247694500382775, 0.045247694500382775, 0.045247694500382775, 0.6846573169729425, 0.6846573169729425, 0.6846573169729425, 0.6846573169729425}, {0.06266177140471169, 0.06266177140471169, 0.06266177140471169, 0.06266177140471169, 0.5091910067230891, 0.5091910067230891, 0.5091910067230891, 0.5091910067230891}, {0.08016732334164174, 0.08016732334164174, 0.08016732334164174, 0.08016732334164174, 0.32434342321250337, 0.32434342321250337, 0.32434342321250337, 0.32434342321250337}, {0.091932882326037, 0.091932882326037, 0.091932882326037, 0.091932882326037, 0.10671194840788394, 0.10671194840788394, 0.10671194840788394, 0.10671194840788394}, {0.11471118539925268, 0.11471118539925268, 0.11471118539925268, 0.11471118539925268, -0.12221485846378785, -0.12221485846378785, -0.12221485846378785, -0.12221485846378785}, {0.13380261262755627, 0.13380261262755627, 0.13380261262755627, 0.13380261262755627, -0.1101202332569859, -0.1101202332569859, -0.1101202332569859, -0.1101202332569859}, {0.15311012579147015, 0.15311012579147015, 0.15311012579147015, 0.15311012579147015, 0.02857223334470323, 0.02857223334470323, 0.02857223334470323, 0.02857223334470323}, {0.17262679999999905, 0.17262679999999905, 0.17262679999999905, 0.17262679999999905, -0.026589199911371875, -0.026589199911371875, -0.026589199911371875, -0.026589199911371875}, {0.1696616057362413, 0.1696616057362413, 0.1696616057362413, 0.1696616057362413, 0.09192166589838945, 0.09192166589838945, 0.09192166589838945, 0.09192166589838945}, {0.14946914366316486, 0.14946914366316486, 0.14946914366316486, 0.14946914366316486, 0.07649652311010587, 0.07649652311010587, 0.07649652311010587, 0.07649652311010587}, {0.12854159999999834, 0.12854159999999834, 0.12854159999999834, 0.12854159999999834, -0.017950274699994665, -0.017950274699994665, -0.017950274699994665, -0.017950274699994665}, {0.1086057999999994, 0.1086057999999994, 0.1086057999999994, 0.1086057999999994, -0.01494161707287134, -0.01494161707287134, -0.01494161707287134, -0.01494161707287134}, {0.08996351522967358, 0.08996351522967358, 0.08996351522967358, 0.08996351522967358, -0.1946322226946833, -0.1946322226946833, -0.1946322226946833, -0.1946322226946833}, {0.07725821375413663, 0.07725821375413663, 0.07725821375413663, 0.07725821375413663, -0.49731800029766904, -0.49731800029766904, -0.49731800029766904, -0.49731800029766904}, {0.05093995690025627, 0.05093995690025627, 0.05093995690025627, 0.05093995690025627, -0.32790321829266794, -0.32790321829266794, -0.32790321829266794, -0.32790321829266794}, {0.028568599999998417, 0.028568599999998417, 0.028568599999998417, 0.028568599999998417, -0.010528990596116563, -0.010528990596116563, -0.010528990596116563, -0.010528990596116563}, {0.03446327558488935, 0.03446327558488935, 0.03446327558488935, 0.03446327558488935, 0.35335570156577917, 0.35335570156577917, 0.35335570156577917, 0.35335570156577917}, {0.039735920082465286, 0.039735920082465286, 0.039735920082465286, 0.039735920082465286, 0.41649042051391627, 0.41649042051391627, 0.41649042051391627, 0.41649042051391627}, {0.016072400000000188, 0.016072400000000188, 0.016072400000000188, 0.016072400000000188, 0.007900858168476077, 0.007900858168476077, 0.007900858168476077, 0.007900858168476077}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}};
        
        public static final double kPDriveToTargetXY = 1;
        public static final double kIDriveToTargetXY = 0.0;
        public static final double kDDriveToTargetXY = 0.0;
        

        public static final double kPDriveToTargetT = 1;
        public static final double kIDriveToTargetT = 0.0;
        public static final double kDDriveToTargetT = 0.0;
        
    }


}