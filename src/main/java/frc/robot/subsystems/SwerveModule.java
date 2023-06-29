
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
  private final double absEncoderOffsetRad;

  public SwerveModule(int driveMotorID, int turnMotorID, int absEncoderID, double absEncoderOffsetRad) {
    this.absEncoderOffsetRad = absEncoderOffsetRad;
    this.absEncoder = new CANCoder(absEncoderID); 

    this.driveMotor = new WPI_TalonFX(driveMotorID);
    this.turnMotor = new WPI_TalonFX(turnMotorID);

    this.turnMotor.setNeutralMode(NeutralMode.Coast);
    this.driveMotor.setNeutralMode(NeutralMode.Brake);

    this.turnPIDController = new PIDController(Constants.Drivetrain.kPTurn, Constants.Drivetrain.kITurn, Constants.Drivetrain.kDTurn);
    turnPIDController.enableContinuousInput(0, (2.0 * Math.PI));

    absEncoder.configMagnetOffset(Math.toDegrees(absEncoderOffsetRad));
    absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    zeroMotorABS();
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

  /**
   * @return Position of drive wheel in meters
  */
  public double getDrivePositionMeters() {
    double circum = 2 * (Math.PI) * (Constants.Measurements.kWheelDiameterMeters / 2);
    double currentRots = driveMotor.getSelectedSensorPosition() / (2048.0 * Constants.Measurements.kDriveMotorGearRatio);
    return currentRots * circum;
  }

  /**
   * @return Turn angle of wheel in radians 0 to 2Pi
   */
  public double getTurnPositionRad() {
    double posRad =  turnMotor.getSelectedSensorPosition() / Constants.Measurements.kTurnMotorGearRatio % 2048 / 2048 * Math.PI * 2;
    if (posRad < 0) {
      posRad += Math.PI * 2;
    }
    return posRad;  
  }
  
  public double getAbsEncoderRad() {
    double angle = absEncoder.getAbsolutePosition();
    return Math.toRadians(angle);
  }

  public void zeroMotorABS() {
    double absRad = getAbsEncoderRad();
    double absPercent = absRad / (Math.PI * 2);
    double absPercentWithGearRatio = absPercent / Constants.Measurements.kTurnMotorGearRatio;
    double encoderPos = absPercentWithGearRatio * 2048.0;
    turnMotor.setSelectedSensorPosition(encoderPos);
  }

  public void zeroMotors() { 
    driveMotor.setSelectedSensorPosition(0);
    turnMotor.setSelectedSensorPosition(0);
  }

  public void zeroDriveMotor() {
    driveMotor.setSelectedSensorPosition(0);
  }




  public SwerveModuleState optimizeState(SwerveModuleState swerveState) {
    double currentRad = getTurnPositionRad(); 
    if (currentRad > Math.PI) {
      currentRad -= (2 * Math.PI); // currentRad from -Pi to Pi
    }
    return SwerveModuleState.optimize(swerveState, new Rotation2d(currentRad)); 
  }

  public void setDesiredState(SwerveModuleState swerveState) {
    SwerveModuleState state = optimizeState(swerveState);

    // Drive Motor Calculations
    double driveSpd = state.speedMetersPerSecond * Constants.Drivetrain.kMaxSpeed;
    if (Constants.Drivetrain.IS_PRECISE) {
      driveSpd *= Constants.Drivetrain.PRECISE_DRIVE_SPEED_PERCENT;
    }
    if (Math.abs(driveSpd) < Constants.Drivetrain.kDriveMotorDeadband) {
      stopMotors(); return;
    }
    if (Constants.Drivetrain.IS_PRECISE) {
      driveSpd /= 3.0;
    }

    
    // Turn motor calculations
    double targetAngleRad = state.angle.getRadians();
    if (targetAngleRad < 0) {
      targetAngleRad += (Math.PI * 2.0); // targetAngleRad from 0 to 2Pi
    }
    double turnSpd = turnPIDController.calculate(getTurnPositionRad(), targetAngleRad);


    // Setters
    driveMotor.set(driveSpd);
    turnMotor.set(ControlMode.PercentOutput, turnSpd);         
  }
  
  public void stopMotors() {
    driveMotor.set(0);
    turnMotor.set(0);
  }
  



  }


