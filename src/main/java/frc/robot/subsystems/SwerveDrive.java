
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive extends SubsystemBase {
    Timer timer;
    double last = 0.0;
    boolean isPrecise = false;
    double targetHeading;
    PIDController headingController = new PIDController(Constants.Drivetrain.kPHeading, Constants.Drivetrain.kIHeading, Constants.Drivetrain.kDHeading);

    SwerveModuleState[] states = {
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0))
    };

    // Init Swerve Modules 
    private final SwerveModule frontLeft = new SwerveModule(
        RobotMap.Drivetrain.FRONT_LEFT_TALON_D,
        RobotMap.Drivetrain.FRONT_LEFT_TALON_T,
        Constants.Drivetrain.FRONT_LEFT_D_IS_REVERSED,
        Constants.Drivetrain.FRONT_LEFT_T_IS_REVERSED,
        RobotMap.Drivetrain.FRONT_RIGHT_ABS_ENCODER_ID,
        Constants.Drivetrain.FRONT_RIGHT_ABS_ENCODER_OFFSET_RAD,
        true);

    private final SwerveModule frontRight = new SwerveModule(
        RobotMap.Drivetrain.FRONT_RIGHT_TALON_D,
        RobotMap.Drivetrain.FRONT_RIGHT_TALON_T,
        Constants.Drivetrain.FRONT_RIGHT_D_IS_REVERSED,
        Constants.Drivetrain.FRONT_RIGHT_T_IS_REVERSED,
        RobotMap.Drivetrain.BACK_RIGHT_ABS_ENCODER_ID,
        Constants.Drivetrain.BACK_RIGHT_ABS_ENCODER_OFFSET_RAD,
        false);

    private final SwerveModule backLeft = new SwerveModule(
        RobotMap.Drivetrain.BACK_LEFT_TALON_D,
        RobotMap.Drivetrain.BACK_LEFT_TALON_T,
        Constants.Drivetrain.BACK_LEFT_D_IS_REVERSED,
        Constants.Drivetrain.BACK_LEFT_T_IS_REVERSED,
        RobotMap.Drivetrain.FRONT_LEFT_ABS_ENCODER_ID,
        Constants.Drivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET_RAD,
        false);

    private final SwerveModule backRight = new SwerveModule(
        RobotMap.Drivetrain.BACK_RIGHT_TALON_D,
        RobotMap.Drivetrain.BACK_RIGHT_TALON_T,
        Constants.Drivetrain.BACK_RIGHT_D_IS_REVERSED,
        Constants.Drivetrain.BACK_RIGHT_T_IS_REVERSED,
        RobotMap.Drivetrain.BACK_LEFT_ABS_ENCODER_ID,
        Constants.Drivetrain.BACK_LEFT_ABS_ENCODER_OFFSET_RAD,
        true);

    // Init gyro
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();
    
    public void zeroHeading() {
        gyro.reset();
        
        // gyro.setYawAxis(null)
    }

    public void zeroTalons() {
        frontLeft.zeroTalon();
        frontRight.zeroTalon();
        backLeft.zeroTalon();
        backRight.zeroTalon();
        frontLeft.zeroDrive();
        frontRight.zeroDrive();
        backLeft.zeroDrive();
        backRight.zeroDrive();
    }
    // public void zeroTalonsABS() {
    //     frontLeft.resetEncoderPos();
    //     frontRight.resetEncoderPos();
    //     backLeft.resetEncoderPos();
    //     backRight.resetEncoderPos();
    // }

    public void zeroDrive() {
        frontLeft.zeroDrive();
        frontRight.zeroDrive();
        backLeft.zeroDrive();
        backRight.zeroDrive();
    }

    public void clearStickyFaults() {
        frontLeft.clearStickyFaults();
        frontRight.clearStickyFaults();
        backLeft.clearStickyFaults();
        backRight.clearStickyFaults();
    }

    public void setCurrentLimitTurn(double currentLimit) {
        frontLeft.setCurrentLimitTurn(currentLimit);
        frontRight.setCurrentLimitTurn(currentLimit);
        backLeft.setCurrentLimitTurn(currentLimit);
        backRight.setCurrentLimitTurn(currentLimit);
    }

    public void setCurrentLimitDrive(double currentLimit) {
        frontLeft.setCurrentLimitDrive(currentLimit);
        frontRight.setCurrentLimitDrive(currentLimit);
        backLeft.setCurrentLimitDrive(currentLimit);
        backRight.setCurrentLimitDrive(currentLimit);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] swerveModulePositions = {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurnPosition())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurnPosition())),
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurnPosition())),
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurnPosition()))
        };
        return swerveModulePositions;
    }

    public void togglePrecise() {
        isPrecise = !isPrecise;
    }
    
    public SwerveDrive() {
        this.timer = new Timer();
        timer.reset();

        // Init gyro with delay
        new Thread(() -> {
            try {
                Thread.sleep(Constants.Drivetrain.GYRO_DELAY_MS);
                zeroHeading();
                calibrateGyro();
                // zeroTalonsABS();
                zeroTalons();
            }
            catch (Exception e) {

            }
        }
        ).start();

    }

    public void setTurnDefaultMode(NeutralMode mode) {
        frontLeft.setTurnDefaultMode(mode);
        frontRight.setTurnDefaultMode(mode);
        backLeft.setTurnDefaultMode(mode);
        backRight.setTurnDefaultMode(mode);
    }

    public void setDriveDefaultMode(NeutralMode mode) {
        frontLeft.setDriveDefaultMode(mode);
        frontRight.setDriveDefaultMode(mode);
        backLeft.setDriveDefaultMode(mode);
        backRight.setDriveDefaultMode(mode);
    }
    
    public void calibrateGyro() {
        gyro.calibrate();
    }

    public double getDrivePosition() {
        return (frontLeft.getDrivePosition() + frontRight.getDrivePosition() + backLeft.getDrivePosition() + backRight.getDrivePosition()) / 4;
    }

    public double getPitchDegFiltered() {
        double deg = gyro.getYComplementaryAngle();
        deg -= 360;
        deg %= 360;
        deg = Math.abs(deg);

        if (deg > 180) { 
            deg = 0;
        }
        return deg;
    
    }

    public double getPitchDeg() {
        double deg = gyro.getYComplementaryAngle();
        deg %= 360;
        if (deg >= 180) {
            deg -= 360;
        }

        double defaultOffset = -1.8;
        deg -= defaultOffset;

        return deg;
    
    }

    // Get gyro angle from 0 to 360
    public double getHeading() {
        double angle = gyro.getAngle() % 360.0;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    public double getHeadingRad() {
        double angle = gyro.getAngle() % 360.0;
        if (angle < 0) {
            angle += 360;
        }
        angle = Math.toRadians(angle);
        
        return angle;
    }

    
    // Stop modules
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // Set module speeds/angles
    public void setModuleStates(double xSpeed, double ySpeed, double tSpeed) {
        double diff = Math.abs(timer.get() - last); // Adjust for exec time for consistent turning 
        last = timer.get();
        
        this.targetHeading += tSpeed;
        headingController.setSetpoint(targetHeading);
        double tSpeedPID = headingController.calculate(getHeading());
            
        double headingRad = Math.toRadians(-getHeading());
        if (Constants.Auton.FACING_DRIVERS) {
            headingRad += Math.PI;
        }

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, tSpeedPID, new Rotation2d(headingRad));


        // Calculate module states per module
        SwerveModuleState[] moduleStates = Constants.Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Normalize speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.kMaxSpeedMetersPerSecond);
        
        frontLeft.setDesiredState(moduleStates[0], isPrecise);
        frontRight.setDesiredState(moduleStates[1], isPrecise);
        backLeft.setDesiredState(moduleStates[2], isPrecise);
        backRight.setDesiredState(moduleStates[3], isPrecise);

        frontLeft.clearStickyFaults();
        frontRight.clearStickyFaults();
        backLeft.clearStickyFaults();
        backRight.clearStickyFaults();


        // Debug
        // SmartDashboard.putNumber("FrontLeftAngleTarget", desiredStates[0].angle.getDegrees());
        // SmartDashboard.putNumber("FrontRightAngleTarget", desiredStates[1].angle.getDegrees());
        // SmartDashboard.putNumber("BackLeftAngleTarget", desiredStates[2].angle.getDegrees());
        // SmartDashboard.putNumber("BackRightAngleTarget", desiredStates[3].angle.getDegrees());

        // SmartDashboard.putNumber("FrontLeftAngleABS", Math.toDegrees(frontLeft.getAbsEncoderRad()));
        // SmartDashboard.putNumber("FrontRightAngleABS", Math.toDegrees(frontRight.getAbsEncoderRad()));
        // SmartDashboard.putNumber("BackLeftAngleABS", Math.toDegrees(backLeft.getAbsEncoderRad()));
        // SmartDashboard.putNumber("BackRightAngleABS", Math.toDegrees(backRight.getAbsEncoderRad()));

        SmartDashboard.putNumber("FrontLeftAngleTalonEncoder", Math.toDegrees(frontLeft.getTurnPosition()));
        SmartDashboard.putNumber("FrontRightAngleTalonEncoder", Math.toDegrees(frontRight.getTurnPosition()));
        SmartDashboard.putNumber("BackLeftAngleTalonEncoder", Math.toDegrees(backLeft.getTurnPosition()));
        SmartDashboard.putNumber("BackRightAngleTalonEncoder", Math.toDegrees(backRight.getTurnPosition()));

        SmartDashboard.putNumber("Gyro", getHeading());

        

    }

    public double getFrontLeftTicks() {
        return frontLeft.getDrivePosition();
    }

    public SwerveModuleState[] getDesiredStates() {
        return states;
    }

    
    // // Odometer

    // public SwerveModulePosition[] getModulePositions() {
        // return new SwerveModulePosition[] {
            // new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurnPosition())),
            // new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurnPosition())),
            // new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurnPosition())),
            // new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurnPosition())),
        // };
    // }
    // private SwerveModulePosition[] swerveModulePositions = {
    //     new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurnPosition())),
    //     new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurnPosition())),
    //     new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurnPosition())),
    //     new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurnPosition())),
    // };

    // private final SwerveDriveOdometry odometer = new SwerveDriveOdometry( 
    //     Constants.Drivetrain.driveKinematics, new Rotation2d(getHeading()), getModulePositions()); 

    
    // // Update Odometer
    // @Override
    // public void periodic() {
    //     odometer.update(new Rotation2d(Math.toRadians(getHeading())), getModulePositions());
    // }

    // // Get odometer position (pose2d contains x, y, theta in translation/rotation 2d)
    // public Pose2d getPose() {
    //     return odometer.getPoseMeters();
    // }

    // // Reset odometer TODO: Check getting odometry pose method vs getting actual pose
    // public void resetOdometer() {
    //     odometer.resetPosition(new Rotation2d(Math.toRadians(getHeading())),
    //     getModulePositions(),
    //     getPose());
    // }




}