
package frc.robot.subsystems;

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
  private final boolean absEncoderIsReversed;
  private final double absEncoderOffsetRad;


  public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorIsReversed, boolean turnMotorIsReversed, 
  int absEncoderID, double absEncoderOffsetRad, boolean absEncoderIsReversed) {
  
  // public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorIsReversed, boolean turnMotorIsReversed) {
    
    this.absEncoderIsReversed = absEncoderIsReversed;
    this.absEncoderOffsetRad = absEncoderOffsetRad;
    // absEncoder = new edu.wpi.first.wpilibj.AnalogInput(absEncoderID);
    absEncoder = new CANCoder(absEncoderID); 


    // driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    // turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    driveMotor = new WPI_TalonFX(driveMotorID);
    turnMotor = new WPI_TalonFX(turnMotorID);


    driveMotor.setInverted(driveMotorIsReversed);
    turnMotor.setInverted(turnMotorIsReversed);

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
      return Constants.Drivetrain.kTicks2Radians(turnMotor.getSelectedSensorPosition());
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
      return angle;
    }

    public void resetEncoderPos() {
      // driveMotor.setSelectedSensorPosition(Constants.Drivetrain.turnDefaultOffset);
      // turnMotor.setSelectedSensorPosition(Constants.Drivetrain.driveDefaultOffset);
      driveMotor.setSelectedSensorPosition(0);
      turnMotor.setSelectedSensorPosition(getAbsEncoderRad());
    }

    public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
      if (Math.abs(state.speedMetersPerSecond) < 0.005) {
        stop();
        return;
      }

      state = SwerveModuleState.optimize(state, getState().angle);
      // Standard Drive
      // driveMotor.set(state.speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSecond);

      // PID Drive
      // driveMotor.set(drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond));
      driveMotor.set(state.speedMetersPerSecond);

      // turnMotor.set(turnPIDController.calculate(getTurnPosition(), state.angle.getRadians()));
      turnMotor.set(state.angle.getRadians());
    }

    public void stop() {
      driveMotor.set(0);
      turnMotor.set(0);
    }
    



  }




