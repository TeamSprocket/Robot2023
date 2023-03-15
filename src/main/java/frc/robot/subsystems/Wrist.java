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

    private double output = 0;

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

    public void moveWrist(double output) {
        wrist.set(output); 
        this.output = output;
    }

    public double getOutput() {
        return output;
    }

    public void setWristAngle(double currentAngle, double setpoint){
        double output = wristPIDController.calculate(currentAngle, setpoint);
        if (output < -0.3){
            output = -0.3;
        } else if (output > 0.3) {
            output = 0.3;
        }
        wrist.set(output);
    }

    public void stop() {
        wrist.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("[Wrist] Position", wristEncoder.getPosition()); 
        SmartDashboard.putNumber("[Wrist] current", wrist.getOutputCurrent()); 

    }
}