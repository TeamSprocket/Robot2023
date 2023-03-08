
package frc.robot.subsystems;

import javax.sound.sampled.TargetDataLine;

import com.ctre.phoenix.motorcontrol.ControlMode;
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


    driveMotor.setInverted(driveMotorIsReversed);
    turnMotor.setInverted(turnMotorIsReversed); 

    this.isTurnedReverse = isTurnedReverse;

    turnMotor.setNeutralMode(NeutralMode.Coast);
    driveMotor.setNeutralMode(NeutralMode.Brake);

    turnPIDController = new PIDController(Constants.Drivetrain.kPTurn, Constants.Drivetrain.kITurn, Constants.Drivetrain.kDTurn);
    // drivePIDController = new PIDController(Constants.Drivetrain.kPDrive, Constants.Drivetrain.kITurn, 0);

    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoderPos();

  }


    public double getDrivePosition() {
      double circum = 2 * (Math.PI) * (Constants.Drivetrain.kWheelDiameterMeters / 2);
      double currentRots = driveMotor.getSelectedSensorPosition() / (2048.0 * Constants.Drivetrain.kDriveMotorGearRatio);
      return currentRots * circum;
    }

    public double getTurnPosition() {
      double currentPosDeg = turnMotor.getSelectedSensorPosition() / (2048.0 * Constants.Drivetrain.kTurningMotorGearRatio) * 360.0;
      double currentPosRad = Math.toRadians(currentPosDeg);
      // if (currentPosRad > Math.PI) {
      //   currentPosRad = -1.0 * ((2.0 * Math.PI) - currentPosDeg);
      // }
      return currentPosRad;

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
        turnMotor.setSelectedSensorPosition((getAbsEncoderRad() / (Math.PI * 2)) * (2048.0 * Constants.Drivetrain.kTurningMotorGearRatio)); 
      // turnMotor.setSelectedSensorPosition(getAbsEncoderRad() / (2.0 * Math.PI) * 2048.0);
    }

    public void zeroTalon() {
      // driveMotor.setSelectedSensorPosition(0);
      turnMotor.setSelectedSensorPosition(0);
    }


    public void setDesiredState(SwerveModuleState state, double xSpeed, double ySpeed) {
      // SwerveModuleState state = SwerveModuleState.optimize(swerveState, new Rotation2d(getTurnPosition() % Math.PI));
      double currentDeg = Math.toDegrees(getTurnPosition());
      double targetDeg = state.angle.getDegrees();
      double driveSpd = state.speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSecond;

      if (Math.abs(currentDeg % 360 - targetDeg) > 90) {
        targetDeg = targetDeg - 180.0;
        driveSpd *= -1.0;
      }


      if (Math.abs(state.speedMetersPerSecond /  Constants.Drivetrain.kMaxSpeedMetersPerSecond) < 0.01) {
        stop();
        return;
      }

        
      
      if (isTurnedReverse && (xSpeed < 0.01 || ySpeed < 0.01)) {
        SmartDashboard.putNumber("XSPEED", xSpeed);
        driveMotor.set(-driveSpd);
      } else {
        driveMotor.set(driveSpd);
      }

      if (Constants.Drivetrain.SWERVE_IS_SLOW) {
        driveSpd /= 3.0;
      }
      driveMotor.set(driveSpd);

      double turnOutput = turnPIDController.calculate(getTurnPosition() % Math.PI, Math.toRadians(targetDeg));
        // double turnOutput = turnPIDController.calculate(getTurnPosition() % Math.PI, sate.angle.getRadians());
      turnMotor.set(ControlMode.PercentOutput, turnOutput); 

      SmartDashboard.putNumber("CurrentPID", Math.toDegrees(getTurnPosition()));
      SmartDashboard.putNumber("TargetPID", Math.toDegrees(state.angle.getRadians()));
      SmartDashboard.putNumber("DrivespdPID", state.speedMetersPerSecond);

    }

    
    public void stop() {
      driveMotor.set(0);
      turnMotor.set(0);
    }
    



  }



