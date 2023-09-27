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

    private final CANSparkMax wrist = new CANSparkMax(RobotMap.Wrist.WRIST, MotorType.kBrushless);
    
    private PIDController wristPIDController = new PIDController(Constants.Wrist.P, Constants.Wrist.I, Constants.Wrist.D);
    
    private RelativeEncoder wristEncoder = wrist.getEncoder();
    
    public Wrist() {
        wrist.restoreFactoryDefaults();

        wrist.setInverted(false);

        wrist.setSmartCurrentLimit(105);

    }

    public double getWristPosition(){
        return wristEncoder.getPosition();
    }

    public double getWristAngle(){
        double angle = wristEncoder.getPosition() * Constants.Wrist.angleConversionFactor;
        return angle;
    }

    public void resetWristEncoder(){
        wrist.getEncoder().setPosition(0);
    }

    public void moveWrist(double output) {
        wrist.set(output); 
    }

    public void setWristAngle(double setpoint){
        double output = wristPIDController.calculate(getWristAngle(), setpoint);
        if (output < -0.275){
            output = -0.275;
        } else if (output > 0.275) {
            output = 0.275;
        }
        wrist.set(output);
    }

    public void setWristAngle(double __, double setpoint){
        double output = wristPIDController.calculate(getWristAngle(), setpoint);
        if (output < -0.275){
            output = -0.275;
        } else if (output > 0.275) {
            output = 0.275;
        }
        wrist.set(output);
    }

    public void stop() {
        wrist.stopMotor();
    }

    public void clearStickyFaults() {
        wrist.clearFaults();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("[Wrist] Position", wristEncoder.getPosition()); 
        SmartDashboard.putNumber("[Wrist] current", wrist.getOutputCurrent()); 

    }
}