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
import frc.util.Conversions;
// import frc.util.PIDPlus;


public class Elevator extends SubsystemBase{

    public ElevatorState state = ElevatorState.OFF;  // TODO: Change back to HOME
    public enum ElevatorState {
        CLEAR,
        HOME, 
        LOW_CONE, 
        MID_CONE, 
        HIGH_CONE, 
        LOW_CUBE,
        MID_CUBE,
        HIGH_CUBE,
        INTAKE_CUBE,
        INTAKE_CONE,
        INTAKE_FLOOR_CONE,
        MANUAL,
        OFF
    }
    
    private CANSparkMax elevatorLeft = new CANSparkMax(RobotMap.Elevator.ELEVATOR_LEFT, MotorType.kBrushless);
    private CANSparkMax elevatorRight = new CANSparkMax(RobotMap.Elevator.ELEVATOR_RIGHT, MotorType.kBrushless);

    private RelativeEncoder elevatorLeftEncoder = elevatorLeft.getEncoder();
    private RelativeEncoder elevatorRightEncoder = elevatorRight.getEncoder();

    // private final PIDPlus pidController = new PIDPlus(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);
    private final PIDController pidController = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);
    

    public Elevator() {
        elevatorLeft.restoreFactoryDefaults();
        elevatorRight.restoreFactoryDefaults();

        elevatorLeft.setInverted(false);
        elevatorRight.setInverted(true);

        elevatorRight.follow(elevatorLeft, true); 
        
        elevatorLeft.enableVoltageCompensation(8);
        elevatorRight.enableVoltageCompensation(8);

        pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorHOME);
        // pidController.setMinMax(-Constants.Elevator.kMaxSpeed, Constants.Elevator.kMaxSpeed);

        setEncoderHomeOffset();
    }


    @Override
    public void periodic() {
        // System.out.println(state.toString());

        switch (state) {
            
            // no manual/off yet :(
            case CLEAR:
                // System.out.println("Going to clear height");
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorCLEAR);
                break;
            case HOME:
                System.out.println("Going to home height");
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorHOME);
                break;
            case LOW_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorLOW_CONE);
                break;
            case MID_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorMID_CONE);
                break;
            case HIGH_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorHIGH_CONE);
                break;
            case LOW_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorLOW_CUBE);
                break;
            case MID_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorMID_CUBE);
                break;
            case HIGH_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorHIGH_CUBE);
                break;
            case INTAKE_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorINTAKE_CUBE);
                break;
            case INTAKE_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorINTAKE_CONE);
                break;
            case INTAKE_FLOOR_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kElevatorINTAKE_FLOOR_CONE);
                break;

            case MANUAL:
                // idk
                break;
            case OFF:
                // idk
                break;
        }

        double val = pidController.calculate(getElevatorHeight(), pidController.getSetpoint());
        // SmartDashboard.putNumber("Elevator PID Output", getElevatorHeight() - pidController.getSetpoint());

        if (state == ElevatorState.OFF) {
            val = 0;
        } //((getIsOutOfBoundsMin() && val > 0) || (getIsOutOfBoundsMax() && val < 0)) && 

        
        setPercentOutput(val);
        System.out.println(val);
            // System.out.println(pidController.getSetpoint());

        logDebugInfo();
    }


    public void setState(ElevatorState state) {
        this.state = state;
    }

    public void setPercentOutput(double percent) {
        SmartDashboard.putNumber("Elevator Percent Output", percent);
        elevatorLeft.set(percent);
    }

    public ElevatorState getState() {
        return state;
    }

    public boolean reachedStatePos() {
        SmartDashboard.putNumber("Elevator diff height", pidController.getSetpoint());
        return Math.abs(getElevatorHeight() - pidController.getSetpoint()) < Constants.Elevator.reachedStatePosTolerance;
    }

    public boolean reachedClearHeight() {
        return getElevatorHeight() >= Constants.Elevator.clearHeight;
    }

    /**
     * @return Current height of elevator in meters
     */
    public double getElevatorHeight() {
        double height = Conversions.RotationsToMeters(
            elevatorLeftEncoder.getPosition(),
            Constants.Elevator.kElevatorGearRatio,
            Constants.Elevator.kSprocketRadius);
        // height -= Constants.Elevator.homeOffset;
        return height;
    }

    public void setEncoderHomeOffset() {
        elevatorLeftEncoder.setPosition(Constants.Elevator.homeOffset);
    }

    public boolean getIsOutOfBoundsMin() {
        return getElevatorHeight() >= Constants.Elevator.MAX_HEIGHT;
    }

    public boolean getIsOutOfBoundsMax() {
        return getElevatorHeight() <= Constants.Elevator.MIN_HEIGHT;
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
        SmartDashboard.putString("Elevator Height (m)", "" + getElevatorHeight());
        SmartDashboard.putString("Elevator Pos", "" + elevatorLeftEncoder.getPosition());
        
    }

}
