
package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Conversions;
import frc.util.ShuffleboardPIDTuner;

public class SwerveModule extends SubsystemBase {
  
  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turnMotor;
  private final PIDController turnPIDController; 
  private final CANCoder cancoder;
  private Supplier<Double> cancoderOffsetDeg;

  private int rots = 0;

  public SwerveModule(int driveMotorID, int turnMotorID, int cancoderID, Supplier<Double> cancoderOffsetDeg, boolean driveIsReversed) {
    this.driveMotor = new WPI_TalonFX(driveMotorID);
    this.turnMotor = new WPI_TalonFX(turnMotorID);
    this.cancoder = new CANCoder(cancoderID);
    this.cancoderOffsetDeg = cancoderOffsetDeg; 
    
    this.driveMotor.setInverted(driveIsReversed);

    turnPIDController = new PIDController(Constants.Drivetrain.kPTurnMotor, Constants.Drivetrain.kITurnMotor, Constants.Drivetrain.kDTurnMotor);
    turnPIDController.enableContinuousInput(-180, 180);
    turnMotor.setInverted(true);
    
    cancoder.configFactoryDefault();
    CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
    cancoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360; //og, Signed_PlusMinus180
    cancoder.configAllSettings(cancoderConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ResetTicks", Conversions.degreesToFalcon(cancoder.getAbsolutePosition(), Constants.Drivetrain.kTurningMotorGearRatio));
    SmartDashboard.putNumber("TurnPosDeg", getTurnPosition());
    SmartDashboard.putNumber("ABSDeg", getCANCoderDegrees());
    SmartDashboard.putNumber("TurnTicks", turnMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("initialDeg", Conversions.falconToDegrees(turnMotor.getSelectedSensorPosition(), Constants.Drivetrain.kTurningMotorGearRatio));
    double deg = Conversions.falconToDegrees(turnMotor.getSelectedSensorPosition(), Constants.Drivetrain.kTurningMotorGearRatio);
    deg -= (deg >  180) ? 360 : 0;
    SmartDashboard.putNumber("Degree", deg);
    


    // this.cancoderOffsetDeg = (ShuffleboardPIDTuner.get("CancoderOffsetDegCancoderOffsetDegTEMP"));
  }

  /**
   * @return Wheel pos in degrees (-180, 180)
   */
  public double getTurnPosition() {
    double deg = Conversions.falconToDegrees(turnMotor.getSelectedSensorPosition(), Constants.Drivetrain.kTurningMotorGearRatio);
    deg -= (deg >  180) ? 360 : 0; // deg += (deg <  -180) ? 360 : 0 ???
    //deg %= 180;
    return deg;
    //SmartDashboard.putNumber("Degree", deg);
  }

  public double getCANCoderDegrees() {
    double offsetDeg = cancoder.getAbsolutePosition() - cancoderOffsetDeg.get();
    offsetDeg = (offsetDeg < 0) ? (offsetDeg % 360) + 360 : offsetDeg;
    return offsetDeg;
  }

  public void zeroTurnMotorABS() {
    double ticks = Conversions.degreesToFalcon(getCANCoderDegrees(), Constants.Drivetrain.kTurningMotorGearRatio);
    Timer.delay(0.1);
    turnMotor.setSelectedSensorPosition(ticks);
  }

  public void setState(SwerveModuleState moduleState) {
    SwerveModuleState state = SwerveModuleState.optimize(moduleState, new Rotation2d(Math.toRadians(getTurnPosition()))); //check values, might be jank
    // SwerveModuleState state = moduleState;

    double fullTargetAngle = state.angle.getDegrees();
    SmartDashboard.putNumber("fullTargetAngleOptimized", fullTargetAngle);
    
    if (fullTargetAngle < 0) { //useless
      fullTargetAngle += (Math.PI);
    }
    /*
    if (fullTargetAngle > 180) {
      fullTargetAngle -= (360);
    }
    */
    driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond);

    // double turnOutput = turnPIDController.calculate(getTurnPosition(), fullTargetAngle);
    // turnMotor.set(ControlMode.PercentOutput, turnOutput);
    double degs = state.angle.getDegrees();
    /*
    if (degs > 180) {
      degs -= (360);
    }
    */
    turnMotor.set(ControlMode.Position, Conversions.degreesToFalcon(degs, Constants.Drivetrain.kTurningMotorGearRatio));
  

    SmartDashboard.putNumber("Turn Target", state.angle.getDegrees());
    SmartDashboard.putNumber("deg Turn Target", degs);
    
  }

  
}
