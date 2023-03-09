
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

import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.controller.PIDController;


public class SwerveDrive extends SubsystemBase {
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
        // gyro.setYawAxis(null)
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

    public SwerveDrive() {
        // Init gyro with delay
        new Thread(() -> {
            try {
                Thread.sleep(Constants.Drivetrain.GYRO_DELAY_MS);
                zeroHeading();
                // zeroTalonsABS();
                zeroTalons();
            } catch (Exception e) {

            }
        }).start();

    }

    public double getDrivePosition() {
        return (frontLeft.getDrivePosition() + frontRight.getDrivePosition() + backLeft.getDrivePosition()
                + backRight.getDrivePosition()) / 4;
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

    public double getFrontLeftTicks() {
        return frontLeft.getDrivePosition();
    }

    // // Odometer

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurnPosition())),
                new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurnPosition())),
                new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurnPosition())),
                new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurnPosition())),
        };
    }

    private SwerveModulePosition[] swerveModulePositions = {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurnPosition())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurnPosition())),
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurnPosition())),
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurnPosition())),
    };

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
            Constants.Drivetrain.driveKinematics, new Rotation2d(getHeading()), getModulePositions());

    // Update Odometer
    @Override
    public void periodic() {
        odometer.update(new Rotation2d(Math.toRadians(getHeading())), getModulePositions());
    }

    // Get odometer position (pose2d contains x, y, theta in translation/rotation
    // 2d)
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    // how to implement a resetPose position
    // public Pose2d resetPose(){
    // return
    // }

    // Reset odometer TODO: Check getting odometry pose method vs getting actual
    // pose
    public void resetOdometer() {

        odometer.resetPosition(new Rotation2d(Math.toRadians(getHeading())),
                getModulePositions(),
                getPose());
    }

    // Assuming this method is part of a drivetrain subsystem that provides the
    // necessary methods
    
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {


        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        // this.resetOdometry(traj.getInitialHolonomicPose());
                        resetOdometer();
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getPose, // Pose supplier
                        this.kinematics, // SwerveDriveKinematics
                        new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0
                                                    // will only use feedforwards.
                        new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving
                                                    // them 0 will only use feedforwards.
                        this::setModuleStates, // Module states consumer
                        true, // Should the path be automatically mirrored depending on alliance color.
                              // Optional, defaults to true
                        this // Requires this drive subsystem
                ));
    }

}
