
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
  
  private final PIDController turnPIDController; 

  private final CANCoder absEncoder;
  private final double absEncoderOffsetRad;
  
  private final boolean isTurnedReverse;


  public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorIsReversed, boolean turnMotorIsReversed, 
  int absEncoderID, double absEncoderOffsetRad, boolean isTurnedReverse) {
    this.absEncoderOffsetRad = absEncoderOffsetRad;
    absEncoder = new CANCoder(absEncoderID); 

    driveMotor = new WPI_TalonFX(driveMotorID);
    turnMotor = new WPI_TalonFX(turnMotorID);

    turnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    driveMotor.setInverted(driveMotorIsReversed);
    turnMotor.setInverted(turnMotorIsReversed);

    this.isTurnedReverse = isTurnedReverse;

    turnMotor.setNeutralMode(NeutralMode.Coast);

    turnPIDController = new PIDController(Constants.Drivetrain.kPTurn, Constants.Drivetrain.kITurn, Constants.Drivetrain.kDTurn);

    turnPIDController.enableContinuousInput(0, (2.0 * Math.PI));

    resetEncoderPos();

  }


    public double getDrivePosition() {
      double circum = (Math.PI) * Constants.Drivetrain.kWheelDiameterMeters;
      double currentRots = driveMotor.getSelectedSensorPosition() / 2048.0;
      return currentRots * circum;
    }

    public double getTurnPosition() {
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
      double tunedAbsEncoderRad = -getAbsEncoderRad();
        turnMotor.setSelectedSensorPosition(tunedAbsEncoderRad / (2.0 * Math.PI)
               * 2048.0 * Constants.Drivetrain.kTurningMotorGearRatio);
    }

    public void zeroTalon() {
      turnMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState optimizeState(SwerveModuleState swerveState) {
      double currentRad = getTurnPosition();
      if (currentRad > Math.PI) {
        currentRad -= (Math.PI * 2); 
      }
      
      return SwerveModuleState.optimize(swerveState, new Rotation2d(currentRad));
    }


    public void setDesiredState(SwerveModuleState swerveState) {
      SwerveModuleState state = optimizeState(swerveState);
    

      double fullTargetAngle = state.angle.getRadians();
        if (fullTargetAngle < 0) {
          fullTargetAngle += (Math.PI * 2.0);
        }
      double driveSpd = state.speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSecond;

      if (Math.abs(state.speedMetersPerSecond /  Constants.Drivetrain.kMaxSpeedMetersPerSecond) < 0.01) {
        stop();
        return;
      }


      driveMotor.set(driveSpd);

      double offsetAngle = Math.abs(Math.abs((state.angle.getDegrees() + 90.0) % 180.0) 
                  - Math.abs(Math.toDegrees(getAbsEncoderRad()) % 180.0));
      SmartDashboard.putNumber("OFFSET ANGLE", offsetAngle);
      // if (offsetAngle > 10) {
      //           double turnOutput = turnPIDController.calculate(
      //             getAbsEncoderRad(),
      //             ((state.angle.getRadians() + (Math.PI / 2.0)) % Math.PI));

      //           turnMotor.set(ControlMode.PercentOutput, turnOutput);
      // //           turnMotor.set(0.3 * 
      // //           ((state.angle.getDegrees() + 90.0) % 180.0 - Math.toDegrees(getAbsEncoderRad()) % 180.0) /
      // //           (Math.abs(state.angle.getDegrees() + 90.0) % 180.0 - Math.toDegrees(getAbsEncoderRad()) % 180.0));
      // }
      // else {
      //   turnMotor.set(ControlMode.PercentOutput, 0);
      // }
      // double offsetAngle = state.angle.getDegrees() - Math.toDegrees(getAbsEncoderRad());
      // if (offsetAngle >= 3) {
      //   turnMotor.set(ControlMode.PercentOutput, offsetAngle / 180.0);
      // }

                // SmartDashboard.putNumber("CURRENT PID OUTPUT", turnOutput);
      SmartDashboard.putNumber("CURRENT ANGLE", Math.toDegrees(getAbsEncoderRad()));
      SmartDashboard.putNumber("TARGET ANGLE", Math.toDegrees((state.angle.getRadians() + (Math.PI / 2.0)) % Math.PI));
      // turnMotor.set(TalonFXControlMode.PercentOutput, turnOutput);
      // turnMotor.set(ControlMode.MotionMagic, 
      //   (getTurnPosition() + (state.angle.getDegrees() - Math.toDegrees(getAbsEncoderRad()))
      //      * 2048.0 / Constants.Drivetrain.kTurningMotorGearRatio / 360));
      // SmartDashboard.putNumber("Turn Output PID", turnOutput);

      // target angle in deg
      // double targetAngle = ((2048.0 / 360.0) * nearestDeg) / 
      //   (Constants.Drivetrain.kTurningMotorGearRatio)  % 360 / 2048.0;
      // double targetAngle = (nearestDeg - Math.toDegrees(getAbsEncoderRad())) % 360 / (5 * 360);
            
      // SmartDashboard.putNumber("Target Angle", targetAngle);
      // turnMotor.set(TalonFXControlMode.PercentOutput, targetAngle);

      
      // if (getAbsEncoderRad() > state.angle.getDegrees() && 
      // Math.toDegrees(getAbsEncoderRad()) - state.angle.getDegrees() < 10) {
      //   turnMotor.set(
      //     0.1);
      // }
      // else if (Math.toDegrees(getAbsEncoderRad()) < state.angle.getDegrees() && 
      // Math.toDegrees(getAbsEncoderRad()) - state.angle.getDegrees() < 10) {
      //   turnMotor.set(-0.1);
      // }

      // SmartDashboard.putNumber("", absEncoderOffsetRad)
      // turnMotor.set(state.angle.getRadians());
    }

    
    public void stop() {
      driveMotor.set(0);
      turnMotor.set(0);
    }
    



  }



