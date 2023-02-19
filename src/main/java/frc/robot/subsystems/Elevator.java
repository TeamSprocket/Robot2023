package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.peresistent.ElevateJoystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class Elevator extends SubsystemBase{
    
    private CANSparkMax elevatorLeft = new CANSparkMax(RobotMap.Elevator.ELEVATOR_LEFT, MotorType.kBrushless);
    private CANSparkMax elevatorRight = new CANSparkMax(RobotMap.Elevator.ELEVATOR_RIGHT, MotorType.kBrushless);

    private final PIDController elevatorPIDController = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);

    private RelativeEncoder elevatorLeftEncoder = elevatorLeft.getEncoder();
    private RelativeEncoder elevatorRightEncoder = elevatorRight.getEncoder();
    

    public Elevator(){
        elevatorLeft.restoreFactoryDefaults();
        elevatorRight.restoreFactoryDefaults();


        elevatorLeft.setInverted(Constants.Elevator.ELEVATOR_LEFT_IS_INVERTED);


        elevatorRight.follow(elevatorLeft, Constants.Elevator.ELEVATOR_RIGHT_IS_INVERTED);
        
        // elevatorLeft.enableVoltageCompensation(8);
        // elevatorRight.enableVoltageCompensation(8);

        // elevatorPIDControllerller.setOutputRange(Constants.Elevator.kMinOutput, Constants.Elevator.kMaxOutput);

    }


////////////////////////////////////////////////////////////////

    //TODO: Use encoder values for height measurement
    /**
     * @return Returns elevator position in meters
     */
    public double getElevtorHeight(){
        double motorPos = elevatorLeftEncoder.getPosition() * 2 * Math.PI;
        double sprocketPos = motorPos / Constants.Elevator.kElevatorGearRatio;

        double circum = 2 * Math.PI * (Units.inchesToMeters(Constants.Elevator.kSocketDiameterMeters) / 2);
        return sprocketPos * circum;
    }

    public void stop(){
        elevatorLeft.stopMotor();
        elevatorRight.stopMotor();
    }

//////////////////////////////////////////////////


    public double getElevatorOutput(double setpoint) {
        double output = elevatorPIDController.calculate(
            getElevtorHeight(),
            setpoint);
        if (getElevtorHeight() >= (0.8 * Constants.Elevator.HEIGHT_METERS)
            || getElevtorHeight() <= (0.2 * Constants.Elevator.HEIGHT_METERS)) {
                output *= (Math.abs(getElevtorHeight() - setpoint) / Constants.Elevator.HEIGHT_METERS);
            }
        
        //TODO CHECK DIVISION OR MULTIPLICATION
        output *= Constants.Elevator.MAX_SPEED;
        return output;
    }
    public void setElevatorHeight(double setpoint) {
        double output = elevatorPIDController.calculate(
            getElevtorHeight(),
            setpoint);
        if (getElevtorHeight() >= (0.8 * Constants.Elevator.HEIGHT_METERS)
            || getElevtorHeight() <= (0.2 * Constants.Elevator.HEIGHT_METERS)) {
                output *= (Math.abs(getElevtorHeight() - setpoint) / Constants.Elevator.HEIGHT_METERS);
            }
        
        //TODO CHECK DIVISION OR MULTIPLICATION
        output *= Constants.Elevator.MAX_SPEED;
        elevatorLeft.set(output);
    }

    public void setElevatorHeightEncoder(double setpoint) {
        double encoderHeight = elevatorLeftEncoder.getPositionConversionFactor();
        double output = elevatorPIDController.calculate(
            encoderHeight,
            setpoint);
        if (encoderHeight >= (0.8 * Constants.Elevator.MAX_ENCODER_VALUE)
            || encoderHeight <= (0.2 * Constants.Elevator.MIN_ENCODER_VALUE)) {
                output *= (Math.abs(encoderHeight - setpoint) / Constants.Elevator.ENCODER_RANGE);
            }
        
        //TODO CHECK DIVISION OR MULTIPLICATION
        output /= Constants.Elevator.MAX_SPEED;
        elevatorLeft.set(output);
    }
    public double getLeftEncoder(){
        return elevatorLeftEncoder.getPosition();
    }



    @Override
    public void periodic(){
        SmartDashboard.putNumber("Encoder Left Position", elevatorLeftEncoder.getPosition());
        SmartDashboard.putNumber("Encoder Right Position", elevatorRightEncoder.getPosition());


        SmartDashboard.putNumber("Elvator Target Height", ElevateJoystick.getTargetHeight());
        // SmartDashboard.putNumber("Elvator Output", elevatorLeftEncoder.getElevatorOutput());

        SmartDashboard.putNumber("Voltage", elevatorLeft.getBusVoltage());
        SmartDashboard.putNumber("Output", elevatorLeft.getAppliedOutput());



    }
}
