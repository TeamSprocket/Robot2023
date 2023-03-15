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
        
        public static final double kMaxSpeedMetersPerSecond = 0.2;
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

    }

    public static final class Auton { 
        public static final boolean FACING_DRIVERS = true;

        // public static final double kPBalance = 0.08; //0.08
        // public static final double kIBalance = 0;
        // public static final double kDBalance = 0.00;

        public static final double CENTER_OF_MASS_FROMT_BACK_Y_METERS = 0.3898;
        public static final double BUMPER_THICKNESS_Y_METERS = Units.inchesToMeters(3);
        // public static final double LENGTH_OF_BOT_Y_METERS = 0.7112;
        public static final double CHARGING_STATION_TO_CENTER_Y_METERS = 0.965;

        public static final double SPEED_ON_RAMP = 0.019;

        // public static final double[][] ROUTINE = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0019829572248602953, 0.0019829572248602953, 0.0019829572248602953, 0.0019829572248602953, 0.784819334795683, 0.784819334795683, 0.784819334795683, 0.784819334795683}, {0.010850235359774658, 0.010850235359774658, 0.010850235359774658, 0.010850235359774658, 0.7848501276859677, 0.7848501276859677, 0.7848501276859677, 0.7848501276859677}, {0.023566065895610677, 0.023566065895610677, 0.023566065895610677, 0.023566065895610677, 0.7854301210780046, 0.7854301210780046, 0.7854301210780046, 0.7854301210780046}, {0.03384481755352301, 0.03384481755352301, 0.03384481755352301, 0.03384481755352301, 0.7805408550756316, 0.7805408550756316, 0.7805408550756316, 0.7805408550756316}, {0.0417869129100851, 0.0417869129100851, 0.0417869129100851, 0.0417869129100851, 0.8419244895132849, 0.8419244895132849, 0.8419244895132849, 0.8419244895132849}, {0.0467828228688953, 0.0467828228688953, 0.0467828228688953, 0.0467828228688953, 0.965122850857232, 0.965122850857232, 0.965122850857232, 0.965122850857232}, {0.047567906618169904, 0.047567906618169904, 0.047567906618169904, 0.047567906618169904, 0.9753966533655227, 0.9753966533655227, 0.9753966533655227, 0.9753966533655227}, {0.044670362254212344, 0.044670362254212344, 0.044670362254212344, 0.044670362254212344, 1.0841425407594474, 1.0841425407594474, 1.0841425407594474, 1.0841425407594474}, {0.03685752608965852, 0.03685752608965852, 0.03685752608965852, 0.03685752608965852, 1.2086762227774965, 1.2086762227774965, 1.2086762227774965, 1.2086762227774965}, {0.027397708943338775, 0.027397708943338775, 0.027397708943338775, 0.027397708943338775, 1.3795401648780437, 1.3795401648780437, 1.3795401648780437, 1.3795401648780437}, {0.01967992499999989, 0.01967992499999989, 0.01967992499999989, 0.01967992499999989, 1.5531319765520941, 1.5531319765520941, 1.5531319765520941, 1.5531319765520941}, {0.021202574999999693, 0.021202574999999693, 0.021202574999999693, 0.021202574999999693, 1.5561168914050019, 1.5561168914050019, 1.5561168914050019, 1.5561168914050019}, {0.028661099999999797, 0.028661099999999797, 0.028661099999999797, 0.028661099999999797, 1.5545369127674251, 1.5545369127674251, 1.5545369127674251, 1.5545369127674251}, {0.034375, 0.034375, 0.034375, 0.034375, 1.5512127481107523, 1.5512127481107523, 1.5512127481107523, 1.5512127481107523}, {0.029834950000000228, 0.029834950000000228, 0.029834950000000228, 0.029834950000000228, 1.5459340686562448, 1.5459340686562448, 1.5459340686562448, 1.5459340686562448}, {0.022608024999999993, 0.022608024999999993, 0.022608024999999993, 0.022608024999999993, 1.5419467543443794, 1.5419467543443794, 1.5419467543443794, 1.5419467543443794}, {0.01484994999999998, 0.01484994999999998, 0.01484994999999998, 0.01484994999999998, 1.5404815816064061, 1.5404815816064061, 1.5404815816064061, 1.5404815816064061}, {0.007369225000000236, 0.007369225000000236, 0.007369225000000236, 0.007369225000000236, 1.540221412751208, 1.540221412751208, 1.540221412751208, 1.540221412751208}, {0.0, 0.0, 0.0, 0.0, 1.5407628338192805, 1.5407628338192805, 1.5407628338192805, 1.5407628338192805}, {0.0, 0.0, 0.0, 0.0, 1.5407628338192805, 1.5407628338192805, 1.5407628338192805, 1.5407628338192805}, {0.0, 0.0, 0.0, 0.0, 1.5407628338192805, 1.5407628338192805, 1.5407628338192805, 1.5407628338192805}, {0.0, 0.0, 0.0, 0.0, 1.5407628338192805, 1.5407628338192805, 1.5407628338192805, 1.5407628338192805}, {0.010563061617879827, 0.010563061617879827, 0.010563061617879827, 0.010563061617879827, 0.7561956708770537, 0.7561956708770537, 0.7561956708770537, 0.7561956708770537}, {0.016974913818984527, 0.016974913818984527, 0.016974913818984527, 0.016974913818984527, 0.8946194995643634, 0.8946194995643634, 0.8946194995643634, 0.8946194995643634}, {0.02367396208151381, 0.02367396208151381, 0.02367396208151381, 0.02367396208151381, 1.0960963497626235, 1.0960963497626235, 1.0960963497626235, 1.0960963497626235}, {0.01285983772789993, 0.01285983772789993, 0.01285983772789993, 0.01285983772789993, 1.4150020323753811, 1.4150020323753811, 1.4150020323753811, 1.4150020323753811}, {0.005265150000000141, 0.005265150000000141, 0.005265150000000141, 0.005265150000000141, 1.5434492548275032, 1.5434492548275032, 1.5434492548275032, 1.5434492548275032}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}, {0.0, 0.0, 0.0, 0.0, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396, 1.5440556381515396}};
         
        
    }


}