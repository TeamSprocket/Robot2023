package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;


public class Elevator extends SubsystemBase{
    
    private CANSparkMax elevatorLeft = new CANSparkMax(RobotMap.Elevator.ELEVATOR_LEFT, MotorType.kBrushless);
    private CANSparkMax elevatorRight = new CANSparkMax(RobotMap.Elevator.ELEVATOR_RIGHT, MotorType.kBrushless);

    private final PIDController elevatorPIDController = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);

    private RelativeEncoder elevatorLeftEncoder = elevatorLeft.getEncoder();
    private RelativeEncoder elevatorRightEncoder = elevatorRight.getEncoder();
    

    public Elevator(){
        elevatorLeft.restoreFactoryDefaults();
        elevatorRight.restoreFactoryDefaults();


        elevatorLeft.setInverted(true);
        elevatorRight.setInverted(false);

        elevatorRight.follow(elevatorLeft, true); 
        
        elevatorLeft.enableVoltageCompensation(8);
        elevatorRight.enableVoltageCompensation(8);

    }


////////////////////////////////////////////////////////////////
    public double getElevatorHeight(){
        double sprocketPos = elevatorLeftEncoder.getPosition() / Constants.Elevator.kElevatorGearRatio;
        double circum = 2 * Math.PI * Constants.Elevator.kSprocketRadius;

        return sprocketPos * circum;
    }
    
    public void moveElevator(double output){
        System.out.println(output);
        elevatorLeft.set(output);
    }

    public void setElevatorPosition(double currentPosition, double setpoint){
        double output = elevatorPIDController.calculate(currentPosition, setpoint);
        elevatorLeft.set(output);
    }


    public void stop(){
        elevatorLeft.stopMotor();
        elevatorRight.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("[Elvator] Encoder Left", elevatorLeftEncoder.getPosition());
        SmartDashboard.putNumber("[Elvator] Encoder Right ", elevatorRightEncoder.getPosition());


        SmartDashboard.putNumber("[Elvator] Voltage", elevatorLeft.getBusVoltage());
        SmartDashboard.putNumber("[Elvator] Output", elevatorLeft.getAppliedOutput());



    }
}
