
package frc.robot.subsystems;

import javax.sound.sampled.TargetDataLine;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.fasterxml.jackson.databind.util.ISO8601Utils;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule extends SubsystemBase {

  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turnMotor;
  
  private final PIDController turnPIDController; 

  private final CANCoder absEncoder;
  
  public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorIsReversed, boolean turnMotorIsReversed, 
  int absEncoderID, double absEncoderOffsetRad) {

    driveMotor = new WPI_TalonFX(driveMotorID);
    turnMotor = new WPI_TalonFX(turnMotorID);

    driveMotor.setInverted(driveMotorIsReversed);
    turnMotor.setInverted(turnMotorIsReversed); 

    turnMotor.setNeutralMode(NeutralMode.Coast);
    driveMotor.setNeutralMode(NeutralMode.Brake);

    turnPIDController = new PIDController(Constants.Drivetrain.kPTurn, Constants.Drivetrain.kITurn, Constants.Drivetrain.kDTurn);
    turnPIDController.enableContinuousInput(0, (2.0 * Math.PI));

    absEncoder = new CANCoder(absEncoderID); 
    absEncoder.configMagnetOffset(Math.toDegrees(absEncoderOffsetRad));
    absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
  }

  public void setTurnDefaultMode(NeutralMode mode) {
    turnMotor.setNeutralMode(mode);
  }
  public void setDriveDefaultMode(NeutralMode mode) {
    driveMotor.setNeutralMode(mode);
  }

  public void clearStickyFaults() {
    turnMotor.clearStickyFaults();
    driveMotor.clearStickyFaults();
  }
  
  public void setCurrentLimitTurn(double currentLimit) {
    turnMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimit, 1.0));
    turnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit, 1.0));
  }

  public void setCurrentLimitDrive(double currentLimit) {
    driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimit, 1.0));
    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit, 1.0));
  }

  // Meters
  public double getDrivePosition() {
    double circum = 2 * (Math.PI) * (Constants.Measurements.kWheelDiameterMeters / 2);
    double currentRots = driveMotor.getSelectedSensorPosition() / (2048.0 * Constants.Measurements.kDriveMotorGearRatio);
    return currentRots * circum;
  }

  // 0 to 2Pi
  public double getTurnPosition() {
    double posRad =  turnMotor.getSelectedSensorPosition() / Constants.Measurements.kTurningMotorGearRatio % 2048 / 2048 * Math.PI * 2;
    if (posRad < 0) {
      posRad += Math.PI * 2;
    }
    return posRad;
  }

  // 0 to 360
  public double getAbsEncoderRad() {
    double angle = absEncoder.getAbsolutePosition();
    angle = Math.toRadians(angle);
    return angle;
  }

  public void zeroTurnABS() {
    double absAngle = absEncoder.getAbsolutePosition() % 360.0;
    double absPercent = absAngle / 360.0;
    double absPercentWithRatio = absPercent * Constants.Measurements.kTurningMotorGearRatio;
    double encoderPos = absPercentWithRatio * 2048.0;
    turnMotor.setSelectedSensorPosition(encoderPos);
  }

  public void zeroTalons() { 
    driveMotor.setSelectedSensorPosition(0);
    turnMotor.setSelectedSensorPosition(0);
  }

  public SwerveModuleState optimizeState(SwerveModuleState swerveState) {
    double currentRad = getTurnPosition() % (Math.PI * 2);
    if (currentRad > Math.PI) {
      currentRad -= (Math.PI * 2); 
    }
    return SwerveModuleState.optimize(swerveState, new Rotation2d(currentRad));
  }


  public void setDesiredState(SwerveModuleState swerveState) {
    SwerveModuleState state = optimizeState(swerveState);
  

    double targetAngle = state.angle.getRadians();
    if (targetAngle < 0) {
      targetAngle += (Math.PI * 2.0);
    }
    double turnOutput = turnPIDController.calculate(getTurnPosition(), targetAngle);
    turnMotor.set(ControlMode.PercentOutput, turnOutput); 


    double driveSpd = state.speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedDrive;
    if (Constants.isPreciseDrive) {
      driveSpd *= Constants.Drivetrain.PRECISE_DRIVE_SPEED_PERCENT;
    }
    if (Constants.isEnabled) {
      driveSpd *= Constants.kTeleopMultiplier;
    }
    if (Constants.isPreciseDrive) {
      driveSpd /= 3.0;
    }
    driveMotor.set(driveSpd);    
  }

  
  public void stop() {
    driveMotor.set(0);
    turnMotor.set(0);
  }
  

  }


