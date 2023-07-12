
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
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

    public void zeroTalons() {
        frontLeft.zeroTalons();
        frontRight.zeroTalons();
        backLeft.zeroTalons();
        backRight.zeroTalons();
    }

    public void zeroSwerveABS() {
        frontLeft.zeroTurnABS();
        frontRight.zeroTurnABS();
        backLeft.zeroTurnABS();
        backRight.zeroTurnABS();
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

    public void togglePrecise() {
        Constants.isPreciseDrive = !Constants.isPreciseDrive;
    }
    
    public SwerveDrive() {
        // Init gyro with delay
        new Thread(() -> {
            try {
                Thread.sleep(Constants.GYRO_DELAY_MS);
                zeroHeading();
                gyro.calibrate();
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

    // Get gyro angle from 0 to 360
    public double getHeading() {
        double angle = gyro.getAngle() % 360.0;
        if (angle < 0) {
            angle += 360;
        }

        if (Constants.Auton.FACING_DRIVERS) {
            angle += 180.0;
        }

        angle %= 360.0;

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
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

}