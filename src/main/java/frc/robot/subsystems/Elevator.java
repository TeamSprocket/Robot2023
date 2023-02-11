package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.util.Units;
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

    private double elevatorEncoderBase;

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


    public double getElevatorEncoderBase(){
        return elevatorEncoderBase;
    }

    public void setElevatorEncoderBase(double elevatorHeight){
        elevatorEncoderBase = elevatorHeight;
    }


    //TODO: Use encoder values for height measurement
    /**
     * @return Returns elevator position in meters
     */
    public double getElevtorHeightInMeters(){
        double motorPos = elevatorLeftEncoder.getPositionConversionFactor();
        double sprocketPos = motorPos / Constants.Elevator.kElevatorGearRatio;

        double circum = 2 * Math.PI * (Units.inchesToMeters(1.43) / 2);

        return sprocketPos * circum;
    }

    public void stop(){
        elevatorLeft.stopMotor();
        elevatorRight.stopMotor();
    }

//////////////////////////////////////////////////


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Encoder Left Position", elevatorLeftEncoder.getPosition());
        SmartDashboard.putNumber("Encoder Right Position", elevatorRightEncoder.getPosition());

        SmartDashboard.putNumber("Voltage", elevatorLeft.getBusVoltage());
        SmartDashboard.putNumber("Output", elevatorLeft.getAppliedOutput());

    }
}
