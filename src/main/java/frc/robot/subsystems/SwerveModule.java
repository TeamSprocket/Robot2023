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
    // drivePIDController = new PIDController(Constants.Drivetrain.kPDrive, Constants.Drivetrain.kITurn, 0);

    turnPIDController.enableContinuousInput(0, 2 * Math.PI);

    resetEncoderPos();

  }


    public double getDrivePosition() {
      // return driveEncoder.getPosition();
      // SmartDashboard.putNumber("module " + Math.random(), Constants.Drivetrain.kTicks2Radians(driveMotor.getSelectedSensorPosition()));
      return Constants.Drivetrain.kTicks2Radians(driveMotor.getSelectedSensorPosition());
    }
    public double getTurnPosition() {
      // return turnEncoder.getPosition();
      double turnOutput;
      
      SmartDashboard.putNumber("CURRENT WHEEL TICKS", turnMotor.getSelectedSensorPosition() % (2048.0 * Constants.Drivetrain.kTurningMotorGearRatio));
      // double currentPosAbsDeg = turnMotor.getSelectedSensorPosition() % (2048.0 * Constants.Drivetrain.kTurningMotorGearRatio) / 
      //   (2048.0 * Constants.Drivetrain.kTurningMotorGearRatio) * 360.0;
      double currentPosDeg = turnMotor.getSelectedSensorPosition() / (2048.0 * Constants.Drivetrain.kTurningMotorGearRatio) * 360.0;
      return Math.toRadians(currentPosDeg);

    }
    
    public double getAbsEncoderRad() {
      double angle = absEncoder.getAbsolutePosition();
      angle = Math.toRadians(angle);
      angle -= absEncoderOffsetRad;
      return Math.abs(angle % (Math.PI * 2.0));
    }

    public void resetEncoderPos() {
      driveMotor.setSelectedSensorPosition(0);
        turnMotor.setSelectedSensorPosition((getAbsEncoderRad() / (Math.PI * 2)) * (2048.0 * Constants.Drivetrain.kTurningMotorGearRatio)); 
    }


    public void setDesiredState(SwerveModuleState state) {
      double turnOutput;
      // SwerveModuleState state = SwerveModuleState.optimize(defaultState, new Rotation2d(getTurnPosition()));

      if (Math.abs(state.speedMetersPerSecond /  Constants.Drivetrain.kMaxSpeedMetersPerSecond) < 0.01) {
        stop();
        return;
      }

      // double speedMetersPerSec = (2048 * state.speedMetersPerSecond) 
      //   / (10 * 2 * Math.PI * 0.05 * Constants.Drivetrain.kDriveMotorGearRatio);
        driveMotor.set(state.speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSecond);
      // state.speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSecond);

      // PID Drive
      // driveMotor.set(drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond));
      // SmartDashboard.putNumber("DRIVE SPEED", state.speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSecond);
      // driveMotor.set(state.speedMetersPerSecond);

      // double offsetAngle = Math.abs(Math.abs((state.angle.getDegrees() + 90.0) % 180.0) 
      //           - Math.abs(Math.toDegrees(getAbsEncoderRad()) % 180.0));
      // SmartDashboard.putNumber("OFFSET ANGLE", offsetAngle);

      turnOutput = turnPIDController.calculate(
        getTurnPosition() % (2 * Math.PI), // getAbsEncoderRad(),
      state.angle.getRadians());
  

      turnMotor.set(ControlMode.PercentOutput, turnOutput);
                // turnMotor.set(0.1);
      //           turnMotor.set(0.3 * 
      //           ((state.angle.getDegrees() + 90.0) % 180.0 - Math.toDegrees(getAbsEncoderRad()) % 180.0) /
      //           (Math.abs(state.angle.getDegrees() + 90.0) % 180.0 - Math.toDegrees(getAbsEncoderRad()) % 180.0));
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



