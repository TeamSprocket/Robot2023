package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Arm extends SubsystemBase {
    
    private double output = 0;

    private final CANSparkMax armLeft = new CANSparkMax(RobotMap.Arm.ARM_LEFT,MotorType.kBrushless);
    private final CANSparkMax armRight = new CANSparkMax(RobotMap.Arm.ARM_RIGHT, MotorType.kBrushless);

    private PIDController armPIDController = new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD);   
    
    private RelativeEncoder armLeftEncoder = armLeft.getEncoder();
    private RelativeEncoder armRightEncoder = armRight.getEncoder(); 


    public Arm(){
        armLeft.restoreFactoryDefaults();
        armRight.restoreFactoryDefaults();
        
        armLeft.setInverted(false);
        armRight.setInverted(false);

        armRight.follow(armLeft);

    }

    public double getArmAngle(){
        double angle = armLeftEncoder.getPosition() * Constants.Arm.angleConversionFactor;
        return angle;
    }

    public void moveArm(double output){
        armLeft.set(output);
        this.output = output;
    }

    public double getOutput() {
        return output;
    }

    public void setArmAngleManual(double currentAngle, double setpoint){
        double output = armPIDController.calculate(currentAngle, setpoint);
        armLeft.set(output);
    }

    public void setArmAngle(double currentAngle, double setpoint){
        double output = armPIDController.calculate(currentAngle, setpoint);
        if (output < -0.15){
            output = -0.15;
        } else if (output > 0.15) {
            output = 0.15;
        }
        armLeft.set(output);
    }

    public void setArmAngleSpeed(double currentAngle, double setpoint, double limit){
        double output = armPIDController.calculate(currentAngle, setpoint);
        if (output < -limit){
            output = -limit;
        } else if (output > limit) {
            output = limit;
        }
        armLeft.set(output);
    }

    public void stop() {
        armLeft.stopMotor();
        armRight.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("[Arm] Left Encoder Position", armLeftEncoder.getPosition()); 
        SmartDashboard.putNumber("[Arm] Right Encoder Position", armRightEncoder.getPosition());

    }
}

