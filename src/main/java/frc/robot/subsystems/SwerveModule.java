
package frc.robot.subsystems;

import javax.sound.sampled.TargetDataLine;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.fasterxml.jackson.databind.util.ISO8601Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turnMotor;
  
  // private final CANEncoder driveEncoder;
  // private final CANEncoder turnEncoder;

  // private final PIDController drivePIDController;
  private final PIDController turnPIDController; 

  // private final AnalogInput absEncoder;
  private final CANCoder absEncoder;
  private final double absEncoderOffsetRad;
  
  private final boolean isTurnedReverse;


  public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorIsReversed, boolean turnMotorIsReversed, 
  int absEncoderID, double absEncoderOffsetRad, boolean isTurnedReverse) {
  
  // public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorIsReversed, boolean turnMotorIsReversed) {
    this.absEncoderOffsetRad = absEncoderOffsetRad;
    // absEncoder = new edu.wpi.first.wpilibj.AnalogInput(absEncoderID);
    absEncoder = new CANCoder(absEncoderID); 


    // driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    // turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    driveMotor = new WPI_TalonFX(driveMotorID);
    turnMotor = new WPI_TalonFX(turnMotorID);

    turnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    driveMotor.setInverted(driveMotorIsReversed);
    turnMotor.setInverted(turnMotorIsReversed);

    this.isTurnedReverse = isTurnedReverse;

    turnMotor.setNeutralMode(NeutralMode.Coast);

    turnPIDController = new PIDController(Constants.Drivetrain.kPTurn, Constants.Drivetrain.kITurn, Constants.Drivetrain.kDTurn);
    // drivePIDController = new PIDController(Constants.Drivetrain.kPDrive, Constants.Drivetrain.kITurn, 0);

    turnPIDController.enableContinuousInput(0, (2.0 * Math.PI));

    resetEncoderPos();

  }


    public double getDrivePosition() {
      double circum = (Math.PI) * Constants.Drivetrain.kWheelDiameterMeters;
      double currentRots = driveMotor.getSelectedSensorPosition() / 2048.0;
      return currentRots * circum;
    }

    public double getTurnPosition() {
      // double currentPosDeg = turnMotor.getSelectedSensorPosition() / (2048.0 * Constants.Drivetrain.kTurningMotorGearRatio) * 360.0;
      // double currentPosRad = Math.toRadians(currentPosDeg) % (2 * Math.PI);
      // if (currentPosRad > Math.PI) {
      //   currentPosRad -= Math.PI * 2.0;
      // }

      // currentPosRad %= Math.PI;

      // // if (currentPosRad > Math.PI) {
      //   // currentPosRad = -1.0 * ((2.0 * Math.PI) - currentPosRad);
      // // }
      // return currentPosRad;
      // double currentPosRad = turnMotor.getSelectedSensorPosition() % (2048.0) / 2048.0 / Constants.Drivetrain.kTurningMotorGearRatio * (Math.PI * 2);
      // if (currentPosRad < 0) {
      //   currentPosRad += (Math.PI * 2.0);
      // }
      // return currentPosRad;
      double posRad =  turnMotor.getSelectedSensorPosition() / Constants.Drivetrain.kTurningMotorGearRatio % 2048 / 2048 * Math.PI * 2;
      if (posRad < 0) {
        posRad += Math.PI * 2;
      }
      return posRad;  

    }

    public double getTurnPositionTicks() {
      double pos =  turnMotor.getSelectedSensorPosition() / Constants.Drivetrain.kTurningMotorGearRatio % 2048 / 2048 * Math.PI * 2;
      if (pos < 0) {
        pos += Math.PI * 2;
      }
      return pos;
    }
    
    public double getAbsEncoderRad() {
      double angle = absEncoder.getAbsolutePosition();
      angle = Math.toRadians(angle);
      angle -= absEncoderOffsetRad;
      double rad = Math.abs(angle % (Math.PI * 2.0));
      if (rad > Math.PI) {
        rad = -1.0 * (2.0 * Math.PI - rad);
      }
      return rad;

    }

    public void resetEncoderPos() {
      // driveMotor.setSelectedSensorPosition(0);
        // turnMotor.setSelectedSensorPosition((getAbsEncoderRad() / (Math.PI * 2)) * (2048.0 * Constants.Drivetrain.kTurningMotorGearRatio)); 
      double tunedAbsEncoderRad = -getAbsEncoderRad();
      // if (tunedAbsEncoderRad < 0) {
      //   tunedAbsEncoderRad = getAbsEncoderRad() + (2 * Math.PI);
      // }
        turnMotor.setSelectedSensorPosition(tunedAbsEncoderRad / (2.0 * Math.PI)
               * 2048.0 * Constants.Drivetrain.kTurningMotorGearRatio);
    }

    public void zeroTalon() {
      // driveMotor.setSelectedSensorPosition(0);
      turnMotor.setSelectedSensorPosition(0);
    }


    public void setDesiredState(SwerveModuleState state, double xSpeed, double ySpeed, double currentHeading) {
      
      // SwerveModuleState state = SwerveModuleState.optimize(swerveState, null);
      
      double fullTargetAngle = state.angle.getRadians();
        if (fullTargetAngle < 0) {
          fullTargetAngle += (Math.PI * 2.0);
        }
      
      // SwerveModuleState state = SwerveModuleState.optimize(swerveState, new Rotation2d(turnMotor.getSelectedSensorPosition() % (Math.PI * 2)));
      // SwerveModuleState state = SwerveModuleState.optimize(swerveState, new Rotation2d(getTurnPosition()));
      double driveSpd = state.speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSecond;

      // if (Constants.Drivetrain.IS_FIELD_ORIENTED) {
        // targetDeg -= currentHeading;
        // targetDeg = 
      // }

      // if (Math.abs(currentDeg % 360 - targetDeg) > 90) {
      //   targetDeg = targetDeg - 180.0;
      //   driveSpd *= -1.0;
      // }


      if (Math.abs(state.speedMetersPerSecond /  Constants.Drivetrain.kMaxSpeedMetersPerSecond) < 0.01) {
        stop();
        return;
      }

        
      driveMotor.set(driveSpd);


      // if (!Constants.Drivetrain.TURN_MANUAL) {
        // double turnOutput = turnPIDController.calculate(getTurnPosition() % Math.PI, Math.toRadians(targetDeg));

              double turnOutput = turnPIDController.calculate(
                getTurnPosition(),
                fullTargetAngle
                );
              SmartDashboard.putNumber("Target Full Angle AWOIDJFQLWFIAJLWKJF", fullTargetAngle);
              turnMotor.set(ControlMode.PercentOutput, turnOutput); 
              
      // } else {
        // Manual

          // if (targetDeg < 0) {
          //   targetDeg += 360.0;
          // }
          // targetDeg %= 360;

          // double turnOutput = targetDeg / 360 * 2048.0 * Constants.Drivetrain.kTurningMotorGearRatio;
          // SmartDashboard.putNumber("Target Turn Output Deg", turnOutput);
          // turnMotor.set(ControlMode.PercentOutput, turnOutput);

        // turnMotor.set(ControlMode.MotionMagic, ((position * Constants.FALCON_ENCODER_TICKS_PER_ROTATIONS) /
        // Constants.SWERVE_MOTOR_POSITION_CONVERSION_FACTOR) / 360);

      // }

      

      // SmartDashboard.putNumber("CurrentPID", Math.toDegrees(getTurnPosition()));
      // SmartDashboard.putNumber("TargetPID", Math.toDegrees(state.angle.getRadians()));
      // SmartDashboard.putNumber("DrivespdPID", state.speedMetersPerSecond);

    }

    
    public void stop() {
      driveMotor.set(0);
      turnMotor.set(0);
    }
    



  }



