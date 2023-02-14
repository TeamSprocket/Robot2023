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


public class Arm extends SubsystemBase {
    
    private final CANSparkMax armLeft = new CANSparkMax(RobotMap.Arm.ARM_LEFT,MotorType.kBrushless);
    private final CANSparkMax armRight = new CANSparkMax(RobotMap.Arm.ARM_RIGHT, MotorType.kBrushless);

    private PIDController armPIDController = new PIDController(Constants.Arm.P, Constants.Arm.I, Constants.Arm.D);
    
    private RelativeEncoder armLeftEncoder = armLeft.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    private RelativeEncoder armRightEncoder = armRight.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096); 


    public Arm(){
        armLeft.restoreFactoryDefaults();
        armRight.restoreFactoryDefaults();
        
        armLeft.setInverted(false);

        armRight.follow(armLeft, false);

        // armPIDController.setOutputRange(0,0);
    }

    public void setOutputPID(double setpoint){
        double output;
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

    public void setArmPosition(double setpoint){
        double output = armPIDController.calculate(
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

