package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    // --------DON'T TOUCH--------
    public static boolean kIsTeleop = false;
    // ---------------------------

    public static final class Drivetrain {
        public static final boolean IS_FIELD_ORIENTED = true;
        public static final double PRECISE_DRIVE_SPEED_PERCENT = 0.25;

        public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_RAD =  Math.toRadians(0);
        public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(0);
        public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(0);
        public static final double BACK_LEFT_ABS_ENCODER_OFFSET_RAD = Math.toRadians(0);

        // Deadbands
        public static final double kDriveMotorDeadband = 0.01;
        public static final double kDriveTeleopDeadband = 0.05;

        public static final int GYRO_STARTUP_DELAY_MS = 1000;

        // Turn Motor PID
        public static final double kPTurn = 0.6; //0.6
        public static final double kITurn = 0.0000;
        public static final double kDTurn = 0.0015; 
        
        // Max Drive Speeds
        public static final double kMaxSpeed = 0.2; //0.2
        public static final double kMaxAccel = 1;
        public static final double kMaxSpeedAngular = 0.4;
        public static final double kMaxAccelAngular = 1.25;

        // Limelight Align
        public static final double kLimelightAlignP = 0.0075;
        public static final double kLimelightAlignI = 0.0;
        public static final double kLimelightAlignD = 0.00015;


        // -----DON'T TOUCH-----
        public static boolean IS_PRECISE = false;
        // --------------------
    }

    public static final class Elevator {
        public static final double kSprocketRadius = 2.938;
        public static final double kElevatorGearRatio = 9;

        public static final double kP = 0.175;
        public static final double kI = 0;
        public static final double kD = 0.00675;
    }

    public static final class Arm{
        public static final double kArmGearRatio = 31.5;
        public static final double kAngleConversionFactor = 360.0 / kArmGearRatio;
        
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0.0003;
    }

    public static final class Wrist {
        public static final double kWristGearRatio = 2.75;
        public static final double kAngleConversionFactor = 360.0 / (kWristGearRatio * 10);
        
        public static final double kP = 0.0175;
        public static final double kI = 0;
        public static final double kD = 0.0007;
    }

    public static final class Claw {}

    public static final class PCH {}

    public static final class Auton { 
        public static final boolean FACING_DRIVERS = true;

        public static final double kMaxAccelPerSec = 0.001;
        public static final double kMaxAccelAngular = 0.01;

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

        // -----DON'T TOUCH-----
        public static double DEFAULT_PITCH_OFFSET = 0.0;
        // ---------------------

        // public static final double[][] ROUTINE = {{0.0, 0.0, 0.0, 0.0, 2.356194490192345, -2.356194490192345, 0.7853981633974483, -0.7853981633974483}, {0.0, 0.0, 0.0, 0.0, 2.356194490192345, -2.356194490192345, 0.7853981633974483, -0.7853981633974483}, {0.01632753567810988, 0.01632753567810988, 0.01632753567810988, 0.01632753567810988, 0.5992400782190582, 0.5992400782190582, 0.5992400782190582, 0.5992400782190582}, {0.045247694500382775, 0.045247694500382775, 0.045247694500382775, 0.045247694500382775, 0.6846573169729425, 0.6846573169729425, 0.6846573169729425, 0.6846573169729425}, {0.06266177140471169, 0.06266177140471169, 0.06266177140471169, 0.06266177140471169, 0.5091910067230891, 0.5091910067230891, 0.5091910067230891, 0.5091910067230891}, {0.08016732334164174, 0.08016732334164174, 0.08016732334164174, 0.08016732334164174, 0.32434342321250337, 0.32434342321250337, 0.32434342321250337, 0.32434342321250337}, {0.091932882326037, 0.091932882326037, 0.091932882326037, 0.091932882326037, 0.10671194840788394, 0.10671194840788394, 0.10671194840788394, 0.10671194840788394}, {0.11471118539925268, 0.11471118539925268, 0.11471118539925268, 0.11471118539925268, -0.12221485846378785, -0.12221485846378785, -0.12221485846378785, -0.12221485846378785}, {0.13380261262755627, 0.13380261262755627, 0.13380261262755627, 0.13380261262755627, -0.1101202332569859, -0.1101202332569859, -0.1101202332569859, -0.1101202332569859}, {0.15311012579147015, 0.15311012579147015, 0.15311012579147015, 0.15311012579147015, 0.02857223334470323, 0.02857223334470323, 0.02857223334470323, 0.02857223334470323}, {0.17262679999999905, 0.17262679999999905, 0.17262679999999905, 0.17262679999999905, -0.026589199911371875, -0.026589199911371875, -0.026589199911371875, -0.026589199911371875}, {0.1696616057362413, 0.1696616057362413, 0.1696616057362413, 0.1696616057362413, 0.09192166589838945, 0.09192166589838945, 0.09192166589838945, 0.09192166589838945}, {0.14946914366316486, 0.14946914366316486, 0.14946914366316486, 0.14946914366316486, 0.07649652311010587, 0.07649652311010587, 0.07649652311010587, 0.07649652311010587}, {0.12854159999999834, 0.12854159999999834, 0.12854159999999834, 0.12854159999999834, -0.017950274699994665, -0.017950274699994665, -0.017950274699994665, -0.017950274699994665}, {0.1086057999999994, 0.1086057999999994, 0.1086057999999994, 0.1086057999999994, -0.01494161707287134, -0.01494161707287134, -0.01494161707287134, -0.01494161707287134}, {0.08996351522967358, 0.08996351522967358, 0.08996351522967358, 0.08996351522967358, -0.1946322226946833, -0.1946322226946833, -0.1946322226946833, -0.1946322226946833}, {0.07725821375413663, 0.07725821375413663, 0.07725821375413663, 0.07725821375413663, -0.49731800029766904, -0.49731800029766904, -0.49731800029766904, -0.49731800029766904}, {0.05093995690025627, 0.05093995690025627, 0.05093995690025627, 0.05093995690025627, -0.32790321829266794, -0.32790321829266794, -0.32790321829266794, -0.32790321829266794}, {0.028568599999998417, 0.028568599999998417, 0.028568599999998417, 0.028568599999998417, -0.010528990596116563, -0.010528990596116563, -0.010528990596116563, -0.010528990596116563}, {0.03446327558488935, 0.03446327558488935, 0.03446327558488935, 0.03446327558488935, 0.35335570156577917, 0.35335570156577917, 0.35335570156577917, 0.35335570156577917}, {0.039735920082465286, 0.039735920082465286, 0.039735920082465286, 0.039735920082465286, 0.41649042051391627, 0.41649042051391627, 0.41649042051391627, 0.41649042051391627}, {0.016072400000000188, 0.016072400000000188, 0.016072400000000188, 0.016072400000000188, 0.007900858168476077, 0.007900858168476077, 0.007900858168476077, 0.007900858168476077}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}, {0.0, 0.0, 0.0, 0.0, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462, 0.007293827738426462}};
    }

    public static final class Measurements {
        // Drivetrain
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double DIST_MODULE_OFFSET = 0.572;
        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics (
            new Translation2d(DIST_MODULE_OFFSET / 2, DIST_MODULE_OFFSET / 2),
            new Translation2d(-DIST_MODULE_OFFSET / 2, DIST_MODULE_OFFSET / 2), 
            new Translation2d(DIST_MODULE_OFFSET / 2, -DIST_MODULE_OFFSET / 2),
            new Translation2d(-DIST_MODULE_OFFSET / 2, -DIST_MODULE_OFFSET / 2)
        );

        public static final double kDriveMotorGearRatio = 6.75;
        public static final double kTurnMotorGearRatio = 21.35;

    }

   


}