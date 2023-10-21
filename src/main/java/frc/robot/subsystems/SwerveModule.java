
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
import edu.wpi.first.networktables.DoublePublisher;
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
  
  // private final boolean isTurnedReverse;


  public SwerveModule(int driveMotorID, int turnMotorID,
  // boolean driveMotorIsReversed, boolean turnMotorIsReversed, 
  int absEncoderID, double absEncoderOffsetRad
  ) {
    this.absEncoderOffsetRad = absEncoderOffsetRad;
    absEncoder = new CANCoder(absEncoderID); 

    driveMotor = new WPI_TalonFX(driveMotorID);
    turnMotor = new WPI_TalonFX(turnMotorID);

    turnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    turnMotor.setNeutralMode(NeutralMode.Coast);
    turnMotor.setInverted(true);
    driveMotor.setNeutralMode(NeutralMode.Brake);

    turnPIDController = new PIDController(Constants.Drivetrain.kPTurnMotor, Constants.Drivetrain.kITurnMotor, Constants.Drivetrain.kDTurnMotor);
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

    
    // absEncoder.configMagnetOffset(-Math.toDegrees(absEncoderOffsetRad));
    absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    // absEncoder.configMagnetOffset(Math.toDegrees(absEncoderOffsetRad));

    

    // resetEncoderPos();

  }

    public double getTurnPositionRad() {
      // double posRad =  turnMotor.getSelectedSensorPosition() / Constants.Drivetrain.kTurningMotorGearRatio % 2048 / 2048 * Math.PI * 2;
      // if (posRad > 0) {
      //   posRad += Math.PI * 2;
      // }

      // posRad = Math.abs((2.0 * Math.PI) - posRad);

      // if (posRad > Math.PI) {
      //   posRad -= (Math.PI * 2.0);
      // }
      double posRad = turnMotor.getSelectedSensorPosition() / Constants.Drivetrain.kTurningMotorGearRatio / 2048 % 1 * (2 * Math.PI);
      posRad = Math.abs(posRad);

      if (posRad > Math.PI) {
        posRad -= (Math.PI * 2.0);
      }
      return posRad;  

    }

    
    public double getAbsEncoderRad() {
      double angleDeg = absEncoder.getAbsolutePosition();
      return Math.toRadians(angleDeg);

    }



    public double getTurnResetTicks() {
      double angleRad = (360.0 - absEncoder.getAbsolutePosition());
      angleRad = Math.toRadians(angleRad);
      angleRad -= absEncoderOffsetRad;
      if (angleRad < 0) {
        angleRad += (2.0 * Math.PI);
      }
      double turnPerc = angleRad / (2.0 * Math.PI);
      double turnPercWithRatio = turnPerc * Constants.Drivetrain.kTurningMotorGearRatio;
      double turnTicksWithRatio = turnPercWithRatio * 2048.0;
      return turnTicksWithRatio;
    }
    public void resetTurnABS() {
      // turnMotor.setSelectedSensorPosition(getTurnResetTicks());
      // turnMotor.setSelectedSensorPosition(0);
      // turnMotor.getSensorCollection().setIntegratedSensorPosition(getTurnResetTicks(), 0);

      double pos = degreesToFalcon(Math.toDegrees(getAbsEncoderRad() - Constants.Drivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET_RAD), Constants.Drivetrain.kTurningMotorGearRatio);
      turnMotor.setSelectedSensorPosition(pos);
    }




    public double degreesToFalcon(double degrees, double gearRatio) {
      return degrees / (360.0 / (gearRatio * 2048.0));
  }


    public void setDesiredState(SwerveModuleState swerveState) {
      double driveSpd = swerveState.speedMetersPerSecond / Constants.Drivetrain.kMaxSpeed;
      if (Math.abs(driveSpd) > 0.01) {
        driveMotor.set(driveSpd);
      }

      
      SmartDashboard.putNumber("Gay2", swerveState.angle.getRadians());
      double turnOutput = -1.0 * turnPIDController.calculate(getTurnPositionRad(), swerveState.angle.getRadians());
      // turnMotor.set(turnOutput); 
      SmartDashboard.putNumber("Gay3", turnOutput);
    }


    @Override
    public void periodic() {
      SmartDashboard.putNumber("Gay1", getTurnPositionRad());
    }





  }


