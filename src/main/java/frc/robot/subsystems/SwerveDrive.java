
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.ShuffleboardPIDTuner;

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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive extends SubsystemBase {
    // Init Swerve Modules 
    private final SwerveModule frontLeft = new SwerveModule(
        RobotMap.Drivetrain.FRONT_LEFT_TALON_D,
        RobotMap.Drivetrain.FRONT_LEFT_TALON_T,
        RobotMap.Drivetrain.FRONT_LEFT_ABS_ENCODER_ID,
        Constants.Drivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET_RAD
        // true
        );


    // Init gyro
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();
    

    public SwerveDrive() {
        // Init gyro with delay
        new Thread(() -> {
            try {
                Thread.sleep(Constants.kGyroInitDelayMS);
               gyro.reset();
            }
            catch (Exception e) {

            }
        }
        ).start();

    }




    public void zeroTurnABS() {
        frontLeft.resetTurnABS();
    }

    public double getTurnResetTicks() {
        return frontLeft.getTurnResetTicks();
    }

    // Set module speeds/angles
    public void setModuleSpeeds(double xSpeed, double ySpeed, double tSpeed) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, tSpeed, new Rotation2d(gyro.getAngle()));

        // Calculate module states per module
        SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Normalize speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.kMaxSpeed);
        
        frontLeft.setDesiredState(moduleStates[0]);
    }


    public void resetTurnABS() {
        frontLeft.resetTurnABS();
    }



    public void putDebugInfo() {
        SmartDashboard.putNumber("FrontLeftAngleABS", Math.toDegrees(frontLeft.getAbsEncoderRad()));
        SmartDashboard.putNumber("FrontLeftAngleTalonEncoder", frontLeft.getTurnPositionRad());
        
        SmartDashboard.putNumber("FrontLeftResetTicks", frontLeft.getTurnResetTicks());

    }


    @Override
    public void periodic() {
        // zeroTurnABS();
        putDebugInfo();

    }













}