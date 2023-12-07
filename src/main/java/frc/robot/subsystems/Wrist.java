package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final CANSparkMax wrist = new CANSparkMax(RobotMap.Wrist.WRIST, MotorType.kBrushless);
    
  private PIDController wristPIDController = new PIDController(Constants.Wrist.P, Constants.Wrist.I, Constants.Wrist.D);
  
  private RelativeEncoder wristEncoder = wrist.getEncoder();

  public Wrist() {
    wrist.restoreFactoryDefaults();

        wrist.setInverted(false);

        wrist.setSmartCurrentLimit(999);
        resetWristEncoder();
  }

  public double getEncoderValues() {
    return (wristEncoder.get() * kEncoderTick2Meter);
  }

  public double getWristAngle(){
    double angle = wristEncoder.getPosition() * Constants.Wrist.angleConversionFactor;
    return angle;
  }

  public void resetWristEncoder(){
    wrist.getEncoder().setPosition(0.1);
  }

  public void setWrist(double position) {
    wrist.set(position);
  }

  public void stopMovement() {
    wrist.stopMotor();
  }

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void setMotor(double speed) {
    wristMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator encoder value", getEncoderValues());
    SmartDashboard.putNumber("Wrist current value", wrist.getOutputCurrent()); 
  }
}
