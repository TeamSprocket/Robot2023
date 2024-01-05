
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.ShuffleboardPIDTuner;

import frc.robot.subsystems.SwerveModule.*;

public class SwerveDrive extends SubsystemBase {

  double targetHeading;
  double xSpeed, ySpeed, tSpeed;
  PIDController headingController = new PIDController(Constants.Drivetrain.kPHeading, Constants.Drivetrain.kIHeading, Constants.Drivetrain.kDHeading);

  public static enum Directions {
    FORWARD,
    LEFT, 
    RIGHT,
    BACK
  } 

  
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();
  
  private final SwerveModule frontLeft = new SwerveModule(
        RobotMap.Drivetrain.FRONT_LEFT_TALON_D,
        RobotMap.Drivetrain.FRONT_LEFT_TALON_T,
        RobotMap.Drivetrain.FRONT_LEFT_ABS_ENCODER_ID,
        () -> Constants.Drivetrain.kCANCoderOffsetFrontLeft,
        Constants.Drivetrain.FRONT_LEFT_D_IS_REVERSED
  );
  private final SwerveModule frontRight = new SwerveModule(
        RobotMap.Drivetrain.FRONT_RIGHT_TALON_D,
        RobotMap.Drivetrain.FRONT_RIGHT_TALON_T,
        RobotMap.Drivetrain.FRONT_RIGHT_ABS_ENCODER_ID,
        () -> Constants.Drivetrain.kCANCoderOffsetFrontRight,
        Constants.Drivetrain.FRONT_RIGHT_D_IS_REVERSED
  );
  private final SwerveModule backLeft = new SwerveModule(
        RobotMap.Drivetrain.BACK_LEFT_TALON_D,
        RobotMap.Drivetrain.BACK_LEFT_TALON_T,
        RobotMap.Drivetrain.BACK_LEFT_ABS_ENCODER_ID,
        () -> Constants.Drivetrain.kCANCoderOffsetBackLeft,
        Constants.Drivetrain.BACK_LEFT_D_IS_REVERSED
  );
  private final SwerveModule backRight = new SwerveModule(
        RobotMap.Drivetrain.BACK_RIGHT_TALON_D,
        RobotMap.Drivetrain.BACK_RIGHT_TALON_T,
        RobotMap.Drivetrain.BACK_RIGHT_ABS_ENCODER_ID,
        () -> Constants.Drivetrain.kCANCoderOffsetBackRight,
        Constants.Drivetrain.BACK_RIGHT_D_IS_REVERSED
  );


  public SwerveDrive() {
    headingController.enableContinuousInput(0, 360);
    // ShuffleboardPIDTuner.addSlider("CancoderOffsetDegFL", -360, 360, 0.0);
    // ShuffleboardPIDTuner.addSlider("CancoderOffsetDegFR", -360, 360, 0.0);
    // ShuffleboardPIDTuner.addSlider("CancoderOffsetDegBL", -360, 360, 0.0);
    // ShuffleboardPIDTuner.addSlider("CancoderOffsetDegBR", -360, 360, 0.0);
  }

  @Override
  public void periodic() {
    if (Constants.isEnabled){
      double targetHeading = Math.toRadians(getHeading());
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, tSpeed, new Rotation2d(targetHeading));   
      SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.kMaxSpeed);
      setModuleStates(moduleStates);
    }
  }

  public void setModuleSpeeds(double xSpeed, double ySpeed, double tSpeed) {
    this.targetHeading += tSpeed * Constants.Drivetrain.kMaxTurnSpeed;
    this.targetHeading = (targetHeading % 360.0);
    this.targetHeading = (targetHeading < 0) ? targetHeading + 360.0 : targetHeading;
    headingController.setSetpoint(targetHeading);
    double tSpeedPID = headingController.calculate(getHeading());

    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.tSpeed = tSpeedPID;
}


  /**
   * @return Heading in degrees (0, 360)
   */

  public void setHeading(Directions direction){
    switch (direction) {
      case FORWARD:
        targetHeading = 0;

      case BACK:
        targetHeading = 180;

      case LEFT:
        targetHeading = 90;

      case RIGHT:
        targetHeading = 270;
    }
    }
  
  public double getHeading() { 
    targetHeading = gyro.getAngle() + 180.0;
    
    targetHeading %= 360.0;
    if (targetHeading < 0) {
      targetHeading += 360;
    }

    targetHeading *= (Math.PI / 180.0);

    return targetHeading;
  }
  

  public void initGyro() {
    gyro.reset();
    gyro.calibrate();
    gyro.reset();
  }

  public void zeroGyro() {
    gyro.reset();
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }

  public void resetModulesToAbsolute() {
    frontLeft.zeroTurnMotorABS();
    Timer.delay(0.1);
    frontRight.zeroTurnMotorABS();
    Timer.delay(0.1);
    backLeft.zeroTurnMotorABS();
    Timer.delay(0.1);
    backRight.zeroTurnMotorABS();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setState(desiredStates[0]);//currently setting 
    frontRight.setState(desiredStates[1]);
    backLeft.setState(desiredStates[2]);
    backRight.setState(desiredStates[3]);
  }

}











