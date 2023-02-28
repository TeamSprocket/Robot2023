package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class PCH {
        public static final double NORMALIZED_SUPPLY_VOLTAGE = 0; // TODO: Figure out the value
        public static final int MAX_PSI = 120;
        public static final int MIN_PSI = 80;
    }

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
        public static double kD = 0.000002;
        public static double FF = 0;

    }

    public static final class Wrist {
        
        public static double kWristGearRatio = 2.75;

        public static double angleConversionFactor = 360.0 / kWristGearRatio;
        
        public static double P = 0.0017500;
        public static double I = 0;
        public static double D = 0;
        public static double FF = 0;

        public static double maxAngle;
        public static double minAngle;
        public static double angleRange;

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
        public static final double ENCODER_DISTANCE_PER_PULSE = (WHEEL_DIAMETER_METERS * Math.PI)
                / (double) ENCODER_CPR;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        public static final double ksVolts = 0.654;
        public static final double kvVoltSecondsPerMeter = 2.76;
        public static final double kaVoltSecondsSquaredPerMeter = 0.593;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 1.82;
        public static final double kTrackwidthMeters = 0.7112;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class Shooter {
        public static double MAX_RPM = 2000;
        public static double BOTTOM_TARGET_RPM = 1230; // closer to intake (front of bot)
        public static double BOTTOM_SECOND_TARGET_RPM = 1410;
        public static double TOP_TARGET_RPM = 750; // closer to battery (back of bot)

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

    
}
