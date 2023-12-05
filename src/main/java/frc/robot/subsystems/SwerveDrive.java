
package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.ShuffleboardPIDTuner;

public class SwerveDrive extends SubsystemBase {
  
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
    // ShuffleboardPIDTuner.addSlider("CancoderOffsetDegFL", -360, 360, 0.0);
    // ShuffleboardPIDTuner.addSlider("CancoderOffsetDegFR", -360, 360, 0.0);
    // ShuffleboardPIDTuner.addSlider("CancoderOffsetDegBL", -360, 360, 0.0);
    // ShuffleboardPIDTuner.addSlider("CancoderOffsetDegBR", -360, 360, 0.0);
  }

  @Override
  public void periodic() {
    // Constants.Drivetrain.kCANCoderOffsetFrontLeft = ShuffleboardPIDTuner.get("CancoderOffsetDegFL");
    // Constants.Drivetrain.kCANCoderOffsetFrontRight = ShuffleboardPIDTuner.get("CancoderOffsetDegFR");
    // Constants.Drivetrain.kCANCoderOffsetBackLeft = ShuffleboardPIDTuner.get("CancoderOffsetDegBL");
    // Constants.Drivetrain.kCANCoderOffsetBackRight = ShuffleboardPIDTuner.get("CancoderOffsetDegBR");
  }


  /**
   * @return Heading in degrees (0, 360)
   */
  public double getHeading() {
    // double angle = gyro.getAngle() % 360.0;
    // if (angle < 0) {
    //     angle += 360;
    // }
    // return angle;
    return 0;
  }
  
  public void zeroGyro() {
    gyro.reset();
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }

  public void resetModulesToAbsolute() {
    frontLeft.zeroTurnMotorABS();
    Timer.delay(0.2);
    frontRight.zeroTurnMotorABS();
    Timer.delay(0.2);
    backLeft.zeroTurnMotorABS();
    Timer.delay(0.2);
    backRight.zeroTurnMotorABS();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setState(desiredStates[0]);//currently setting 
    SmartDashboard.putNumber("DriveSpdFL", desiredStates[0].speedMetersPerSecond);

    frontRight.setState(desiredStates[1]);
    backLeft.setState(desiredStates[2]);
    backRight.setState(desiredStates[3]);
  }

}











