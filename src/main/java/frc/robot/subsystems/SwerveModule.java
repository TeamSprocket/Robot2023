
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
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


  public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorIsReversed, boolean turnMotorIsReversed, 
  int absEncoderID, double absEncoderOffsetRad) {
  
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

    turnMotor.setNeutralMode(NeutralMode.Coast);

    turnPIDController = new PIDController(Constants.Drivetrain.kPTurn, Constants.Drivetrain.kITurn, Constants.Drivetrain.kDTurn);
    // drivePIDController = new PIDController(Constants.Drivetrain.kPDrive, 0, 0);

    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoderPos();

  }


    public double getDrivePosition() {
      // return driveEncoder.getPosition();
      // SmartDashboard.putNumber("module " + Math.random(), Constants.Drivetrain.kTicks2Radians(driveMotor.getSelectedSensorPosition()));
      return Constants.Drivetrain.kTicks2Radians(driveMotor.getSelectedSensorPosition());
    }
    public double getTurnPosition() {
      // return turnEncoder.getPosition();
      return turnMotor.getSelectedSensorPosition() / 2048.0 / (150/ 7 / 1) * 360;
      // return getAbsEncoderRad()
    }
    public double getDriveVelocity() {
      // return driveEncoder.getVelocity();
      return Constants.Drivetrain.kTicks2Radians(driveMotor.getSelectedSensorVelocity());
    }
    public double getTurnVelocity() {
      // return turnEncoder.getVelocity();
      return Constants.Drivetrain.kTicks2Radians(turnMotor.getSelectedSensorVelocity());
    } 
    
    
    public double getAbsEncoderRad() {
      double angle = absEncoder.getAbsolutePosition();
      angle = Math.toRadians(angle);
      angle -= absEncoderOffsetRad;
      return angle % 180;
    }

    public void resetEncoderPos() {
      // driveMotor.setSelectedSensorPosition(Constants.Drivetrain.turnDefaultOffset);
      // turnMotor.setSelectedSensorPosition(Constants.Drivetrain.driveDefaultOffset);
      driveMotor.setSelectedSensorPosition(0);
      turnMotor.setSelectedSensorPosition(getAbsEncoderRad());
    }

    public Rotation2d getAngle() {
      return new Rotation2d(
        (2 * Math.PI / (2048 / Constants.Drivetrain.kTurningMotorGearRatio)) * 
        (turnMotor.getSelectedSensorPosition() % (2048 / Constants.Drivetrain.kTurningMotorGearRatio))
      );
    }

    public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }


    public void setDesiredState(SwerveModuleState state) {
      if (Math.abs(state.speedMetersPerSecond) < 0.005) {
        stop();
        return;
      }

      state = SwerveModuleState.optimize(state, getAngle());
      // Standard Drive
      // double speedMetersPerSec = (2048 * state.speedMetersPerSecond) 
      //   / (10 * 2 * Math.PI * 0.05 * Constants.Drivetrain.kDriveMotorGearRatio);
      driveMotor.set(state.speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSecond);
      // state.speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSecond);

      // PID Drive
      // driveMotor.set(drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond));
      // driveMotor.set(state.speedMetersPerSecond);

      double turnOutput = turnPIDController.calculate(
        Math.toDegrees(getAbsEncoderRad()), state.angle.getDegrees());
      turnMotor.set(ControlMode.PercentOutput, turnOutput);
      // turnMotor.set(TalonFXControlMode.PercentOutput, turnOutput);
      // turnMotor.set(ControlMode.MotionMagic, 
      //   (getTurnPosition() + (state.angle.getDegrees() - Math.toDegrees(getAbsEncoderRad()))
      //      * 2048.0 / Constants.Drivetrain.kTurningMotorGearRatio / 360));
      // SmartDashboard.putNumber("Turn Output PID", turnOutput);
      SmartDashboard.putNumber("ABS Radians PID", getAbsEncoderRad());
      SmartDashboard.putNumber("State radians", state.angle.getRadians());
      

      double nearestDeg = Math.round(state.angle.getDegrees());
      // target angle in deg
      // double targetAngle = ((2048.0 / 360.0) * nearestDeg) / 
      //   (Constants.Drivetrain.kTurningMotorGearRatio)  % 360 / 2048.0;
      // double targetAngle = (nearestDeg - Math.toDegrees(getAbsEncoderRad())) % 360 / (5 * 360);
      SmartDashboard.putNumber("State Angle", nearestDeg);
      // SmartDashboard.putNumber("Target Angle", targetAngle);
      // turnMotor.set(TalonFXControlMode.PercentOutput, targetAngle);

      
      // if (Math.toDegrees(getAbsEncoderRad()) > state.angle.getDegrees() && 
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




