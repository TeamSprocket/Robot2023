package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    //TODO Edit PID values, tune speeds
    public static final class Elevator {
    
        //check sprocket radius in inches
        public static double kSprocketRadius = 2.938;
        public static double kElevatorGearRatio = 9;

        //height between base & starting config
        public static double offset = 33.638;

        public static double kP = 0.175;
        public static double kI = 0;
        public static double kD = 0.0075;
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
        public static double kD = 0.00075;
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
        
        public static final double kMaxSpeedMetersPerSecond = 0.2;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1;
        
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 0.3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 0.75;

        public static final double kPhysicalMaxSpeedMetersPerSecond = kMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;   
        
        public static double kTicks2Radians(double ticks) {
            return Math.toRadians((ticks / 2048.0 / 21.4) * 360);
        }

        public static boolean SWERVE_IS_SLOW = false;

        public static final double kLimelightP = 0.01;
        public static final double kLimelightI = 0.0;
        public static final double kLimelightD = 0.0;

    }

    public static final class Auton { 
        public static final boolean FACING_DRIVERS = true;

        public static final double kPBalance = 0.1; //0.08
        public static final double kIBalance = 0;
        public static final double kDBalance = 0.00;

        public static final double CENTER_OF_MASS_FROMT_BACK_Y_METERS = 0.3898;
        public static final double BUMPER_THICKNESS_Y_METERS = Units.inchesToMeters(3);
        // public static final double LENGTH_OF_BOT_Y_METERS = 0.7112;
        public static final double CHARGING_STATION_TO_CENTER_Y_METERS = 0.965;

        public static final double SPEED_ON_RAMP = 0.019;

        public static final double[][] ROUTINE = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05323795226792649, 0.05, -0.096875, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05325451742631391, 0.05, -0.115625, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05324830549135167, 0.05, -0.1171875, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05324519952484248, 0.05, -0.140625, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.053243128879639084, 0.05, -0.15000000000000002, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4251968562602997, 0.05, -0.16718750000000002, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.05, -0.18281250000000002, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.9842519760131836, 0.05, -0.1859375, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.05, -0.1859375, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.05, -0.1859375, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.05, -0.19062500000000002, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.05, -0.1921875, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.05, -0.19375, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.05, -0.19375, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.05, -0.19375, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.9842519760131836, 0.05, -0.1546875, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.052775160407053635, 0.05, 0.03464567065238953, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05279793779198323, 0.05, 0.1291338562965393, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.052802079149777394, 0.05, 0.1574803113937378, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.052802079149777394, 0.05, 0.14645669460296631, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.052802079149777394, 0.05, 0.09921259880065919, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.052802079149777394, 0.05, 0.1322834610939026, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.052802079149777394, 0.05, 0.14488189220428468, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.052802079149777394, 0.05, 0.061417323350906376, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1015625, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1375, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.16875, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1703125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1625, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1453125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.115625, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1125, 1.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1125, 1.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1125, 1.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.11406250000000001, 1.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.11406250000000001, 1.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.115625, 1.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.115625, 1.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.115625, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.115625, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.115625, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.115625, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.11875000000000001, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1484375, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.17500000000000002, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1796875, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1796875, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.18906250000000002, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.1953125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05280311446849136, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.15748031437397003, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.23622047901153564, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.31496062874794006, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3779527544975281, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3937007784843445, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3937007784843445, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4645669162273407, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.053324919402754765, 0.05, -0.2, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05329385967545917, 0.05, -0.1078125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0532772945177197, 0.05, -0.1078125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.053264870648119196, 0.05, -0.1078125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05325451742242618, 0.05, -0.1078125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05324830548746393, 0.05, -0.1078125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05324519952095475, 0.05, -0.1078125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05324209355314965, 0.05, -0.1078125, 0.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05323898758664046, 0.05, -0.1078125, 0.0}};
        

        
    }


}