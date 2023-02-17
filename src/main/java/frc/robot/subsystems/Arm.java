package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;


public class Arm extends SubsystemBase {
    
    private final CANSparkMax armLeft = new CANSparkMax(RobotMap.Arm.ARM_LEFT,MotorType.kBrushless);
    private final CANSparkMax armRight = new CANSparkMax(RobotMap.Arm.ARM_RIGHT, MotorType.kBrushless);

    //CHANGE KS IF FRICTION
    //CHANGE KG UNTIL ARM CAN HOLD ITS POSITION
    //CHANGE KV UNTIL SMOOTH AND SLOW MOTION
    private ArmFeedforward armFeedforward = new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, 
                                                                Constants.Arm.kV, Constants.Arm.kA);

    private PIDController armPIDController = new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD);                                                  

    
    private RelativeEncoder armLeftEncoder = armLeft.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    private RelativeEncoder armRightEncoder = armRight.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096); 


    public Arm(){
        armLeft.restoreFactoryDefaults();
        armRight.restoreFactoryDefaults();
        
        armLeft.setInverted(false);

        armRight.follow(armLeft);

        // armPIDController.setOutputRange(0,0);
    }

    //use encoder values to get the height of the arm
    //choose a specific point on the arm(the end of the arm) to measure the height with the angle
    public double getArmPosition() {
        double motorPos = armLeftEncoder.getPositionConversionFactor();
        double sprocketPos = motorPos / Constants.Elevator.kElevatorGearRatio;

        double armPosRad = sprocketPos * (2 * Math.PI);
        return armPosRad;

        //double circum = 2 * Math.PI * (Units.inchesToMeters(5.731) / 2); 
        // return 2;
    } 

    //TODO Add Offset since 0 degrees is assumed parallel w/ floor - get default angle between the arm and elevator

    public void setArmPositionA(double setpoint){
        double output = armFeedforward.calculate(
            getArmPosition(),
            setpoint);
        //TODO ADD ELEVATOR HEIGHT CHANGES
        


        if (getArmPosition() >= (0.8 * Constants.Arm.HEIGHT_METERS_NO_ELEVATOR)
        || getArmPosition() <= (0.2 * Constants.Arm.HEIGHT_METERS_NO_ELEVATOR)) {
            output *= (Math.abs(getArmPosition() - setpoint) / Constants.Elevator.HEIGHT_METERS);
        }
        
        output /= Constants.Elevator.MAX_SPEED;
        armLeft.set(output);
    }

    public void setArmPosition(double setpoint){
        double output = armPIDController.calculate(
            getArmPosition(),
            setpoint);
        //TODO ADD ELEVATOR HEIGHT CHANGES

        if (getArmPosition() >= (0.8 * Constants.Arm.HEIGHT_METERS_NO_ELEVATOR)
        || getArmPosition() <= (0.2 * Constants.Arm.HEIGHT_METERS_NO_ELEVATOR)) {
            output *= (Math.abs(getArmPosition() - setpoint) / Constants.Elevator.HEIGHT_METERS);
        }
        
        output /= Constants.Arm.MAX_SPEED;
        armLeft.set(output);
    }

    public void stop() {
        armLeft.stopMotor();
        armRight.stopMotor();
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Left Encoder Position", armLeftEncoder.getPosition()); 
        SmartDashboard.putNumber("Arm Right Encoder Position", armRightEncoder.getPosition());

        SmartDashboard.putNumber("Arm Angle", getArmPosition());
    }
}

