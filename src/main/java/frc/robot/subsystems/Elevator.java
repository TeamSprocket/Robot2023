package frc.robot.subsystems;

import org.apache.commons.lang3.Conversion;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Conversions;
import frc.util.PIDPlus;


public class Elevator extends SubsystemBase{

    public ElevatorStates state = ElevatorStates.HOME;
    public enum ElevatorStates {
        CLEAR,
        HOME, 
        LOW_CONE, 
        MID_CONE, 
        HIGH_CONE, 
        LOW_CUBE,
        MID_CUBE,
        HIGH_CUBE,
        IN_CUBE,
        IN_CONE_STANDING,
        IN_CONE_FLOOR,
        MANUAL,
        OFF
    }
    
    private CANSparkMax elevatorLeft = new CANSparkMax(RobotMap.Elevator.ELEVATOR_LEFT, MotorType.kBrushless);
    private CANSparkMax elevatorRight = new CANSparkMax(RobotMap.Elevator.ELEVATOR_RIGHT, MotorType.kBrushless);

    private RelativeEncoder elevatorLeftEncoder = elevatorLeft.getEncoder();
    private RelativeEncoder elevatorRightEncoder = elevatorRight.getEncoder();

    private final PIDPlus pidController = new PIDPlus(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);
    

    public Elevator() {
        elevatorLeft.restoreFactoryDefaults();
        elevatorRight.restoreFactoryDefaults();

        elevatorLeft.setInverted(true);
        elevatorRight.setInverted(false);

        elevatorRight.follow(elevatorLeft, true); 
        
        elevatorLeft.enableVoltageCompensation(8);
        elevatorRight.enableVoltageCompensation(8);

        pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorHOME);
        pidController.setMinMax(-Constants.Elevator.kMaxSpeed, Constants.Elevator.kMaxSpeed);
    }


    @Override
    public void periodic(){
        switch (state) {
            // no manual/off yet :(
            case CLEAR:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorCLEAR);
            case HOME:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorHOME);
            case LOW_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorLOW_CONE);
            case MID_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorMID_CONE);
            case HIGH_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorHIGH_CONE);
            case LOW_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorLOW_CUBE);
            case MID_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorMID_CUBE);
            case HIGH_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorHIGH_CUBE);
            case IN_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorIN_CUBE);
            case IN_CONE_STANDING:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorIN_CONE_STANDING);
            case IN_CONE_FLOOR:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorIN_CONE_FLOOR);

            case MANUAL:
                // idk
            case OFF:
                // idk
        }

        if (getIsInBounds() && state != ElevatorStates.OFF)
            elevatorLeft.set(pidController.calculate(getElevatorHeight()));

        logDebugInfo();
    }


    public void setState(ElevatorStates state) {
        this.state = state;
    }

    public void setPercentOutput(double percent) {
        elevatorLeft.set(percent);
    }

    public ElevatorStates getState() {
        return state;
    }

    public boolean reachedStatePos() {
        return Math.abs(getElevatorHeight() - pidController.getSetpoint()) < Constants.Elevator.reachedStatePosTolerance;
    }

    public boolean reachedClearHeight() {
        return getElevatorHeight() >= Constants.Elevator.clearHeight;
    }

    /**
     * @return Current height of elevator in meters
     */
    public double getElevatorHeight() {
        double height = Conversions.TicksToMeters(
            elevatorLeftEncoder.getPosition() * 2048,
            Constants.Elevator.kElevatorGearRatio,
            Constants.Elevator.kSprocketRadius);
        return height;
    }

    public boolean getIsInBounds() {
        return (getElevatorHeight() >= Constants.Elevator.MIN_HEIGHT && getElevatorHeight() <= Constants.Elevator.MAX_HEIGHT);
    }

    public void resetEncoders() {
        elevatorLeft.getEncoder().setPosition(0);
        elevatorRight.getEncoder().setPosition(0);
    }

    public void clearStickyFaults() {
        elevatorLeft.clearFaults();
        elevatorRight.clearFaults();
    }
    
    public void stop() {
        elevatorLeft.stopMotor();
        elevatorRight.stopMotor();
    }

    public void logDebugInfo() {
        SmartDashboard.putString("Elevator State", state.toString());
    }

}
