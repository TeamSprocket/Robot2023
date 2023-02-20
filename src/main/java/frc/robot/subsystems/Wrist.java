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

        wrist.setInverted(true);

        
    }
    public double getWristHeight(){
        double motorPos = wristEncoder.getPosition() * 2 * Math.PI;
        double sprocketPos = motorPos / Constants.Wrist.kWristGearRatio;

        // double armPosRad = sprocketPos * (2 * Math.PI);
        return sprocketPos;

        //double circum = 2 * Math.PI * (Units.inchesToMeters(5.731) / 2); 
        // return 2;
    }

    public void setWrist(double setpoint) {
        double output = wristPIDController.calculate(getWristHeight(), setpoint);

        output *= (Math.abs(getWristHeight() - setpoint) / Constants.Wrist.ENCODER_RANGE);
        
        output *= Constants.Wrist.MAX_SPEED;
        System.out.println("wrist speed: " + output);
        wrist.set(output);
    }

    
 
    public void stop() {
        wrist.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Wrist Encoder Position", wristEncoder.getPosition()); 

        SmartDashboard.putNumber("Arm Angle", getWristHeight());
    }
}