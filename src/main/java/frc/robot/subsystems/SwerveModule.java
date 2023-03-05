
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
    driveMotor.setNeutralMode(NeutralMode.Brake);

    turnPIDController = new PIDController(Constants.Drivetrain.kPTurn, Constants.Drivetrain.kITurn, Constants.Drivetrain.kDTurn);

    turnPIDController.enableContinuousInput(0, (2.0 * Math.PI));

    resetEncoderPos();

  }


    public double getDrivePosition() {
      double circum = 2 * (Math.PI) * (Constants.Drivetrain.kWheelDiameterMeters / 2);
      double currentRots = driveMotor.getSelectedSensorPosition() / (2048.0 * Constants.Drivetrain.kDriveMotorGearRatio);
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
      driveMotor.setSelectedSensorPosition(0);
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


      if (Constants.Drivetrain.SWERVE_IS_SLOW) {
        driveSpd /= 3.0;
      }
      driveMotor.set(driveSpd);

      double turnOutput = turnPIDController.calculate(
        getTurnPosition(),
        fullTargetAngle
        );
      SmartDashboard.putNumber("Target Full Angle AWOIDJFQLWFIAJLWKJF", fullTargetAngle);
      turnMotor.set(ControlMode.PercentOutput, turnOutput); 
              
    }

    
    public void stop() {
      driveMotor.set(0);
      turnMotor.set(0);
    }
    



  }



