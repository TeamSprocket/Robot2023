package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

    private final CANSparkMax wristMotor = new CANSparkMax(RobotMap.Wrist.WRIST, MotorType.kBrushless);
    
    private SparkMaxPIDController wristPIDController = wristMotor.getPIDController();
    
    private RelativeEncoder wristEncoder = wristMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    
    public Wrist() {
        wristMotor.setInverted(false);

        wristPIDController.setP(Constants.Wrist.P);
        wristPIDController.setI(Constants.Wrist.I);
        wristPIDController.setD(Constants.Wrist.D);
        wristPIDController.setIZone(Constants.Wrist.Iz);
        wristPIDController.setFF(Constants.Wrist.FF);
        wristPIDController.setOutputRange(Constants.Wrist.kMinOutput, Constants.Wrist.kMaxOutput);
    }

    public void setOutputManual(double output) {
        wristMotor.set(output);
    }

    public void setOutputPID(double setpoint) {
        wristPIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }
 
    public void stop() {
        wristMotor.stopMotor();
    }
}