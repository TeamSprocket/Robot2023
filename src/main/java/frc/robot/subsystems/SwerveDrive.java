
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive extends SubsystemBase {

    public enum DriveState{
        TELE,
        VISION,
        LOCKED,
        ODOM_ALIGN,
        AUTO,
        DONE
    }

    private static DriveState driveState = DriveState.TELE;

    final Lock currentAutoTrajectoryLock = new ReentrantLock();
    Trajectory currTrajectory;
    private double autoStartTime;

    // Init Swerve Modules 
    private final SwerveModule frontLeft = new SwerveModule(
        RobotMap.Drivetrain.FRONT_LEFT_TALON_D,
        RobotMap.Drivetrain.FRONT_LEFT_TALON_T,
        Constants.Drivetrain.FRONT_LEFT_D_IS_REVERSED,
        Constants.Drivetrain.FRONT_LEFT_T_IS_REVERSED,
        RobotMap.Drivetrain.FRONT_LEFT_ABS_ENCODER_ID,
        Constants.Drivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET_RAD,
        true);

    private final SwerveModule frontRight = new SwerveModule(
        RobotMap.Drivetrain.FRONT_RIGHT_TALON_D,
        RobotMap.Drivetrain.FRONT_RIGHT_TALON_T,
        Constants.Drivetrain.FRONT_RIGHT_D_IS_REVERSED,
        Constants.Drivetrain.FRONT_RIGHT_T_IS_REVERSED,
        RobotMap.Drivetrain.FRONT_RIGHT_ABS_ENCODER_ID,
        Constants.Drivetrain.FRONT_RIGHT_ABS_ENCODER_OFFSET_RAD,
        false);

    private final SwerveModule backLeft = new SwerveModule(
        RobotMap.Drivetrain.BACK_LEFT_TALON_D,
        RobotMap.Drivetrain.BACK_LEFT_TALON_T,
        Constants.Drivetrain.BACK_LEFT_D_IS_REVERSED,
        Constants.Drivetrain.BACK_LEFT_T_IS_REVERSED,
        RobotMap.Drivetrain.BACK_LEFT_ABS_ENCODER_ID,
        Constants.Drivetrain.BACK_LEFT_ABS_ENCODER_OFFSET_RAD,
        false);

    private final SwerveModule backRight = new SwerveModule(
        RobotMap.Drivetrain.BACK_RIGHT_TALON_D,
        RobotMap.Drivetrain.BACK_RIGHT_TALON_T,
        Constants.Drivetrain.BACK_RIGHT_D_IS_REVERSED,
        Constants.Drivetrain.BACK_RIGHT_T_IS_REVERSED,
        RobotMap.Drivetrain.BACK_RIGHT_ABS_ENCODER_ID,
        Constants.Drivetrain.BACK_RIGHT_ABS_ENCODER_OFFSET_RAD,
        true);

    // Init gyro
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();
    
    public void zeroHeading() {
        gyro.reset();
    }
    public void zeroTalons() {
        frontLeft.zeroTalon();
        frontRight.zeroTalon();
        backLeft.zeroTalon();
        backRight.zeroTalon();
    }
    public void zeroTalonsABS() {
        frontLeft.resetEncoderPos();
        frontRight.resetEncoderPos();
        backLeft.resetEncoderPos();
        backRight.resetEncoderPos();
    }

    public synchronized void setAutoPath(Trajectory trajectory){
        currentAutoTrajectoryLock.lock();
        try{
            // rotController.reset(Odometry.getInstance().getOdometry().getRotation().getRadians());
            // setDriveState(DriveState.AUTO);
            this.currTrajectory = trajectory;
            autoStartTime = Timer.getFPGATimestamp();
        }


        
        finally{
            currentAutoTrajectoryLock.unlock();
        }
    }

    public SwerveDrive() {
        // Init gyro with delay
        new Thread(() -> {
            try {
                Thread.sleep(Constants.Drivetrain.GYRO_DELAY_MS);
                zeroHeading();
                // zeroTalonsABS();
                zeroTalons();
            }
            catch (Exception e) {

            }
        }
        ).start();

    }

    // Get gyro angle from -360 to 360
    public double getHeading() {
        double angle = gyro.getAngle() % 360.0;
        if (angle > 180) {
            angle = -1.0 * (360 - angle);
        }
        return -angle;
    }

    
    // Stop modules
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // Set module speeds/angles
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);


        // Debug
        SmartDashboard.putNumber("FrontLeftAngleTarget", desiredStates[0].angle.getDegrees());
        SmartDashboard.putNumber("FrontRightAngleTarget", desiredStates[1].angle.getDegrees());
        SmartDashboard.putNumber("BackLeftAngleTarget", desiredStates[2].angle.getDegrees());
        SmartDashboard.putNumber("BackRightAngleTarget", desiredStates[3].angle.getDegrees());

        SmartDashboard.putNumber("FrontLeftAngleABS", Math.toDegrees(frontLeft.getAbsEncoderRad()));
        SmartDashboard.putNumber("FrontRightAngleABS", Math.toDegrees(frontRight.getAbsEncoderRad()));
        SmartDashboard.putNumber("BackLeftAngleABS", Math.toDegrees(backLeft.getAbsEncoderRad()));
        SmartDashboard.putNumber("BackRightAngleABS", Math.toDegrees(backRight.getAbsEncoderRad()));

        SmartDashboard.putNumber("FrontLeftAngleTalonEncoder", Math.toDegrees(frontLeft.getTurnPosition()));
        SmartDashboard.putNumber("FrontRightAngleTalonEncoder", Math.toDegrees(frontRight.getTurnPosition()));
        SmartDashboard.putNumber("BackLeftAngleTalonEncoder", Math.toDegrees(backLeft.getTurnPosition()));
        SmartDashboard.putNumber("BackRightAngleTalonEncoder", Math.toDegrees(backRight.getTurnPosition()));

        SmartDashboard.putNumber("Gyro", getHeading());

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
