
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
import frc.util.PIDPlus;


public class Wrist extends SubsystemBase{

    public WristState state = WristState.HOME;
    public enum WristState {
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
    
    private CANSparkMax wristMotor = new CANSparkMax(RobotMap.Wrist.WRIST, MotorType.kBrushless);

    private RelativeEncoder wristEncoder = wristMotor.getEncoder();

    private final PIDPlus pidController = new PIDPlus(Constants.Wrist.kP, Constants.Wrist.kI, Constants.Wrist.kD);
    

    public Wrist() {
        wristMotor.restoreFactoryDefaults();
        wristMotor.setInverted(false);
        
        pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristHOME);
        // pidController.setMinMax(-Constants.Wrist.kMaxSpeed, Constants.Wrist.kMaxSpeed);
        
        setEncoderHomeOffset();
    }


    @Override
    public void periodic(){
        switch (state) {
            // no manual/off yet :(
            case HOME:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristHOME);
                break;
            case LOW_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristLOW_CONE);
                break;
            case MID_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristMID_CONE);
                break;
            case HIGH_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristHIGH_CONE);
                break;
            case LOW_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristLOW_CUBE);
                break;
            case MID_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristMID_CUBE);
                break;
            case HIGH_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristHIGH_CUBE);
                break;
            case INTAKE_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristINTAKE_CUBE);
                break;
            case INTAKE_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristINTAKE_CONE);
                break;
            case INTAKE_FLOOR_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristINTAKE_FLOOR_CONE);
                break;
            
            case MANUAL:
                // idk
                break;
            case OFF:
                // idk
                break;
        }

        // if (getIsInBounds() && state != WristState.OFF)
            // wristMotor.set(pidController.calculate(getWristDegrees()));

        logDebugInfo();
    }


    public void setState(WristState state) {
        this.state = state;
    }

    public void setPercentOutput(double percent) {
        wristMotor.set(percent);
    }

    public WristState getState() {
        return state;
    }

    public boolean reachedStatePos() {
        return Math.abs(getWristDegrees() - pidController.getSetpoint()) < Constants.Wrist.reachedStatePosTolerance;
    }

    public void setEncoderHomeOffset() {
        wristEncoder.setPosition(Constants.Wrist.homeOffset);
    }
    

    /**
     * @return Current height of wrist in meters
     */
    public double getWristDegrees() {
        double angle = Conversions.RotationsToDegrees(wristEncoder.getPosition(), Constants.Wrist.kWristGearRatio);
        return -1 * angle;
    }

    public boolean getIsInBounds() {
        return (getWristDegrees() >= Constants.Wrist.MIN_ANGLE && getWristDegrees() <= Constants.Wrist.MAX_ANGLE);
    }

    public void resetEncoders() {
        wristMotor.getEncoder().setPosition(0);
    }

    public void clearStickyFaults() {
        wristMotor.clearFaults();
    }
    
    public void stop() {
        wristMotor.stopMotor();
    }

    public void logDebugInfo() {
        SmartDashboard.putString("Wrist State", state.toString());
        SmartDashboard.putString("Wrist Angle (degrees)", "" + getWristDegrees());
        SmartDashboard.putString("Wrist Pos", "" + wristEncoder.getPosition());
    }

}

