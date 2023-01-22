
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive extends SubsystemBase {
    // Init Swerve Modules 
    private final SwerveModule frontLeft = new SwerveModule(
        RobotMap.Drivetrain.FRONT_LEFT_TALON_D,
        RobotMap.Drivetrain.FRONT_LEFT_TALON_T,
        Constants.Drivetrain.FRONT_LEFT_D_IS_REVERSED,
        Constants.Drivetrain.FRONT_LEFT_T_IS_REVERSED,
        RobotMap.Drivetrain.FRONT_LEFT_ABS_ENCODER_ID,
        Constants.Drivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET_RAD);

    private final SwerveModule frontRight = new SwerveModule(
        RobotMap.Drivetrain.FRONT_RIGHT_TALON_D,
        RobotMap.Drivetrain.FRONT_RIGHT_TALON_T,
        Constants.Drivetrain.FRONT_RIGHT_D_IS_REVERSED,
        Constants.Drivetrain.FRONT_RIGHT_T_IS_REVERSED,
        RobotMap.Drivetrain.FRONT_RIGHT_ABS_ENCODER_ID,
        Constants.Drivetrain.FRONT_RIGHT_ABS_ENCODER_OFFSET_RAD);

    private final SwerveModule backLeft = new SwerveModule(
        RobotMap.Drivetrain.BACK_LEFT_TALON_D,
        RobotMap.Drivetrain.BACK_LEFT_TALON_T,
        Constants.Drivetrain.BACK_LEFT_D_IS_REVERSED,
        Constants.Drivetrain.BACK_LEFT_T_IS_REVERSED,
        RobotMap.Drivetrain.BACK_LEFT_ABS_ENCODER_ID,
        Constants.Drivetrain.BACK_LEFT_ABS_ENCODER_OFFSET_RAD);

    private final SwerveModule backRight = new SwerveModule(
        RobotMap.Drivetrain.BACK_RIGHT_TALON_D,
        RobotMap.Drivetrain.BACK_RIGHT_TALON_T,
        Constants.Drivetrain.BACK_RIGHT_D_IS_REVERSED,
        Constants.Drivetrain.BACK_RIGHT_T_IS_REVERSED,
        RobotMap.Drivetrain.BACK_RIGHT_ABS_ENCODER_ID,
        Constants.Drivetrain.BACK_RIGHT_ABS_ENCODER_OFFSET_RAD);

    // Init gyro
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();
    public void zeroHeading() {
        gyro.reset();
    }

    // Odometer
    // private final SwerveDriveOdometry odometer = new SwerveDriveOdometry( 
    //     Constants.Drivetrain.driveKinematics, 
    //     gyro.getRotation2d(), 
    //     null); 

    public SwerveDrive() {
        // Init gyro with delay
        new Thread(() -> {
            try {
                Thread.sleep(Constants.Drivetrain.GYRO_DELAY_MS);
                zeroHeading();
            }
            catch (Exception e) {

            }
        }
        ).start();

    }

    // Get gyro angle from -360 to 360
    public double getHeading() {
        return gyro.getAngle() % 360.0;
    }

    // Get rotation as rotation2d
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    } 

    // Update Odometer
    @Override
    public void periodic() {
        // odometer.update(getRotation2d(), SwerveModulePosition(0, gyro.getRotation2d()));
    }

    // Get odometer position (pose2d contains x, y, theta in translation/rotation 2d)
    // public Pose2d getPose() {
    //     return odometer.getPoseMeters();
    // }

    // Reset odometer
    // public void resetOdometer(Pose2d pose) {
        // odometer.resetPosition(gyro.getRotation2d(), getPose(), pose);
    // }

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
        SmartDashboard.putNumber("FrontLeftAngleABS", Math.toDegrees(frontLeft.getAbsEncoderRad()));
        SmartDashboard.putNumber("FrontRightAngleABS", Math.toDegrees(frontRight.getAbsEncoderRad()));
        SmartDashboard.putNumber("BackLeftAngleABS", Math.toDegrees(backLeft.getAbsEncoderRad()));
        SmartDashboard.putNumber("BackRightAngleABS", Math.toDegrees(backRight.getAbsEncoderRad()));

        SmartDashboard.putString("FrontLeftAngle", desiredStates[0].angle.toString());
        SmartDashboard.putString("FrontRightAngle", desiredStates[1].angle.toString());
        SmartDashboard.putString("BackLeftAngle", desiredStates[2].angle.toString());
        SmartDashboard.putString("BackRightAngle", desiredStates[3].angle.toString());

        SmartDashboard.putNumber("Gyro", getHeading());

    }




}
