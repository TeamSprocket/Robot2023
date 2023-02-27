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
    
    private final CANSparkMax armLeft = new CANSparkMax(RobotMap.Arm.ARM_LEFT,MotorType.kBrushless);
    private final CANSparkMax armRight = new CANSparkMax(RobotMap.Arm.ARM_RIGHT, MotorType.kBrushless);

    private PIDController armPIDController = new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD);   
    
    private PIDController armPIDControllerSlow = new PIDController(Constants.Arm.mP, Constants.Arm.mP, Constants.Arm.mP);

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
    }

    public void setArmAngleManual(double currentAngle, double setpoint){
        double output = armPIDController.calculate(currentAngle, setpoint);
        armLeft.set(output);
    }

    public void setArmAngleSlow(double currentAngle, double setpoint){
        double output = armPIDControllerSlow.calculate(currentAngle, setpoint);
        if (output < -0.35){
            output = -0.35;
        } 
        else if (output > 0.35){
            output = 0.35;
        }
        
        armLeft.set(output);
    }

    public void setArmAngle(double currentAngle, double setpoint){
        double output = armPIDController.calculate(currentAngle, setpoint);
        if (output < -0.15){
            output = -0.15;
        } else if (output > 0.15) {
            output = 0.15;
        }
        System.out.println("ANGLE: " + currentAngle);
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

