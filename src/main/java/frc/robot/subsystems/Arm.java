package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Arm extends SubsystemBase {
    
    private final CANSparkMax armLeft = new CANSparkMax(RobotMap.Arm.ARM_LEFT,MotorType.kBrushless);
    private final CANSparkMax armRight = new CANSparkMax(RobotMap.Arm.ARM_RIGHT, MotorType.kBrushless);
    private RelativeEncoder armLeftEncoder = armLeft.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    private RelativeEncoder armRightEncoder = armRight.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096); 
    private SparkMaxPIDController armPIDController = armLeft.getPIDController();

    public Arm(){
        armLeft.setInverted(false);
        armRight.setInverted(false);

        armPIDController.setP(Constants.Arm.P);
        armPIDController.setI(Constants.Arm.I);
        armPIDController.setD(Constants.Arm.D);
        armPIDController.setIZone(Constants.Arm.Iz);
        armPIDController.setFF(Constants.Arm.FF);
        armPIDController.setOutputRange(0,0);


    }

    public void config(){
        armLeft.restoreFactoryDefaults();
        armRight.restoreFactoryDefaults();
    
        armRight.follow(armLeft, true);


        armPIDController.setFeedbackDevice(null);
    }

    public void setOutput(double output) {
        armLeft.set(output);
        armRight.set(output);
    }

    public void setOutputPID(double setpoint){
        armPIDControllersetReference(setpoint, CANSparkMax.ControlType.kPosition);
    }


    
    //use encoder values to get the height of the arm
    //choose a specific point on the arm(the end of the arm) to measure the height with the angle
    public double ArmPosition() {
        //double circum = 2 * Math.PI * (Units.inchesToMeters(5.731) / 2); 
        //double circum = 2 * Math.PI * (0.05371 / 2); 

        return 2;
    }

    public void stop() {
        armLeft.stopMotor();
        armRight.stopMotor();
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Encoder Position", armLeftEncoder.getPosition());
        SmartDashboard.putNumber("Encoder Position", armRightEncoder.getPosition());
    }
}

