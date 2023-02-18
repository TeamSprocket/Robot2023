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
        public static final double DIST_MODULE_OFFSET = 0.572;

        public static final double kDriveMotorGearRatio = 6.75;
        public static final double kTurningMotorGearRatio = 21.35   ;
        
        // public static final double turnDefaultOffset = 0;
        // public static final double driveDefaultOffset = 0;

        public static final boolean BACK_RIGHT_D_IS_REVERSED = false;
        public static final boolean FRONT_RIGHT_D_IS_REVERSED = false;
        public static final boolean BACK_LEFT_D_IS_REVERSED = false;
        public static final boolean FRONT_LEFT_D_IS_REVERSED = false;

        public static final boolean BACK_RIGHT_T_IS_REVERSED = false;
        public static final boolean FRONT_RIGHT_T_IS_REVERSED = false;
        public static final boolean BACK_LEFT_T_IS_REVERSED = false;
        public static final boolean FRONT_LEFT_T_IS_REVERSED = false; 

        public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(115.5);
        public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(-52);
        public static final double BACK_LEFT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(58.5);
        public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_RAD =  Math.toRadians(47);


        // ----------CONST----------
        // public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics (
        //     new Translation2d(-DIST_MODULE_OFFSET / 2, DIST_MODULE_OFFSET / 2),  //FL 
        //     new Translation2d(DIST_MODULE_OFFSET / 2, DIST_MODULE_OFFSET / 2), //FR 
        //     new Translation2d(-DIST_MODULE_OFFSET / 2, -DIST_MODULE_OFFSET / 2), //BL 
        //     new Translation2d(DIST_MODULE_OFFSET / 2, -DIST_MODULE_OFFSET / 2) //BR 
        // );

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

        public static double kTicks2Radians(double ticks) {
            return Math.toRadians((ticks / 2048.0 / 21.4) * 360);
        }

        // ----------TUNED----------
        public static final boolean IS_FIELD_ORIENTED = true;
        public static final boolean TURN_MANUAL = false;
        // public static final double PID_CONST_TEST = 0.04;
        
        // public static final double kPTurn = 0.9; // 0.28
        // public static final double kITurn = 0.0000;
        // public static final double kDTurn = 0.0015; //0.0005?

        public static final double kPTurn = 0.5; // 0.28
        public static final double kITurn = 0.0000;
        public static final double kDTurn = 0.0015; //0.0005?
        

    
        //values that worked best from jason - 0.085, 0.17, 0.0000005    
    
        // public static final double kPTurn = 0.075;
        // public static final double kITurn = 0.000000000000001;
        // public static final double kDTurn = 0.000003;

        // public static final double kPTurn = 0.075; // WORKING (kinda) 
        // public static final double kDTurn = 0.0003; //WORKING (kinda)
        // public static final double kPDrive = PID_CONST_TEST; // module
        // public static final double PID_CONTROLLER_X_P = PID_CONST_TEST; // not module idk
        // public static final double PID_CONTROLLER_Y_P = PID_CONST_TEST; // not module idk
        // public static final double PID_CONTROLLER_T_P = PID_CONST_TEST; // not module id
        
        public static final double kMaxSpeedMetersPerSecond = 0.1;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1;
        
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 0.3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 0.75;

        public static final double kPhysicalMaxSpeedMetersPerSecond = kMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;   
        
    }

    public static final class Arm{
        public static double kS = 0.1;
        public static double kG = 0;
        public static double kV = 0;
        public static double kA = 0;

        public static double kP = 0.1;
        public static double kI = 0;
        public static double kD = 0;
        public static double FF = 0;

        public static double velocityRadPerSec = 0.5;
        public static double accelRadPerSecSquared  = 0.2;

        //Heights listed in meters
        public static double elevatorHeightRestriction = 0.23;

        //TODO UPDATE THIS - all in radians
        public static double armMaxAngle = 1.39;
        public static double armMinAngleNoElevator = 0.75;
        public static double armMinAngleElevator = 0.26;
        // public static double MAX_HEIGHT_METERS_ELEVATOR = 0.15;
        // public static double MIN_HEIGHT_METERS_ELEVATOR = 0.1;
        // public static double HEIGHT_METERS_ELEVATOR = MAX_HEIGHT_METERS_ELEVATOR - MIN_HEIGHT_METERS_ELEVATOR; 


        //TODO UPDATE THIS
        public static double MAX_HEIGHT_METERS_NO_ELEVATOR = 0;
        public static double MIN_HEIGHT_METERS_NO_ELEVATOR = 0;
        public static double HEIGHT_METERS_NO_ELEVATOR = MAX_HEIGHT_METERS_NO_ELEVATOR - MIN_HEIGHT_METERS_NO_ELEVATOR;

        public static double MAX_SPEED = 0.01;
    }


    public static final class Elevator {
        //TODO Edit PID values, check inversions, tune speeds, gear ratio, check if setting encoder base is necessary

        //tune values
        public static double kP = 0.5;
        public static double kI = 0;
        public static double kD = 0.000;
        public static double FF = 0;

        // public static int kMaxOutput = 1;
        // public static int kMinOutput = -1;

        public static boolean ELEVATOR_LEFT_IS_INVERTED = false;
        public static boolean ELEVATOR_RIGHT_IS_INVERTED = true;

        //Heights listed in meters
        //TUNE THE MAX/MIN VALUES
        public static double MAX_HEIGHT_METERS = 10; //0.335
        public static double MIN_HEIGHT_METERS = 0.03;
        public static double HEIGHT_METERS = MAX_HEIGHT_METERS - MIN_HEIGHT_METERS; 

        public static double MAX_ENCODER_VALUE = 23;
        public static double MIN_ENCODER_VALUE = 1;
        public static double ENCODER_RANGE = MAX_ENCODER_VALUE - MIN_ENCODER_VALUE;

        //TODO put units (ex. m/s) & tune speed
        public static double MAX_SPEED = 1.25;

        public static double kElevatorGearRatio = 9; //16.5

        public static int CURRENT_STALL_LIMIT = 30;
        public static int CURRENT_FREE_LIMIT = 30;

        public static double kSocketDiameterMeters = 1.43;
    }
}