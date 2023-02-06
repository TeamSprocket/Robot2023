package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;


public class Elevator extends SubsystemBase{
    
    private final CANSparkMax elevatorLeft = new CANSparkMax(RobotMap.Elevator.ELEVATOR_LEFT, MotorType.kBrushless);
    private final CANSparkMax elevatorRight = new CANSparkMax(RobotMap.Elevator.ELEVATOR_RIGHT, MotorType.kBrushless);

    private SparkMaxPIDController elevatorPIDController = elevatorLeft.getPIDController();

    private RelativeEncoder elevatorLeftEncoder = elevatorLeft.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    private RelativeEncoder elevatorRightEncoder = elevatorRight.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);


    public Elevator(){
        elevatorLeft.setInverted(false);
        elevatorRight.setInverted(false);
        
        elevatorLeft.enableVoltageCompensation(8);
        elevatorRight.enableVoltageCompensation(8);

        elevatorPIDController.setP(Constants.Elevator.P);
        elevatorPIDController.setI(Constants.Elevator.I);
        elevatorPIDController.setD(Constants.Elevator.D);
        elevatorPIDController.setIZone(Constants.Elevator.Iz);
        elevatorPIDController.setFF(Constants.Elevator.FF);
        elevatorPIDController.setOutputRange(Constants.Elevator.kMinOutput, Constants.Elevator.kMaxOutput);

    }

    public void config(){
        elevatorLeft.restoreFactoryDefaults();
        elevatorRight.restoreFactoryDefaults();
    
        elevatorRight.follow(elevatorLeft, true);

        elevatorPIDController.setFeedbackDevice(elevatorLeftEncoder);
    }





/////////////////////////////////////////////////////////////////


    public void setOutputManual(double output){
        elevatorLeft.set(output);
        elevatorRight.set(output);
    }

    public void setOutputPID(double setpoint){
        elevatorPIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }


    //TODO: Use encoder values for height measurement
    public int getElevtorHeight(){
        return 1;
    }

    public void stop(){
        elevatorLeft.stopMotor();
        elevatorRight.stopMotor();
    }



//////////////////////////////////////////////////




    @Override
    public void periodic(){
        SmartDashboard.putNumber("Encoder Position", elevatorLeftEncoder.getPosition());
        SmartDashboard.putNumber("Encoder Position", elevatorRightEncoder.getPosition());
    }

}
