
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


public class Wrist extends SubsystemBase{

    public WristStates state = WristStates.HOME;
    public enum WristStates {
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
    
    private CANSparkMax wristMotor = new CANSparkMax(RobotMap.Wrist.WRIST, MotorType.kBrushless);

    private RelativeEncoder wristEncoder = wristMotor.getEncoder();

    private final PIDPlus pidController = new PIDPlus(Constants.Wrist.kP, Constants.Wrist.kI, Constants.Wrist.kD);
    

    public Wrist() {
        wristMotor.restoreFactoryDefaults();
        wristMotor.setInverted(false);
        
        pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristHOME);
        pidController.setMinMax(-Constants.Wrist.kMaxSpeed, Constants.Wrist.kMaxSpeed);
        
        setEncoderHomeOffset();
    }


    @Override
    public void periodic(){
        switch (state) {
            // no manual/off yet :(
            case HOME:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristHOME);
            case LOW_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristLOW_CONE);
            case MID_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristMID_CONE);
            case HIGH_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristHIGH_CONE);
            case LOW_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristLOW_CUBE);
            case MID_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristMID_CUBE);
            case HIGH_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristHIGH_CUBE);
            case IN_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristIN_CUBE);
            case IN_CONE_STANDING:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristIN_CONE_STANDING);
            case IN_CONE_FLOOR:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kWristIN_CONE_FLOOR);
            
            case MANUAL:
                // idk
            case OFF:
                // idk
        }

        if (getIsInBounds() && state != WristStates.OFF)
            wristMotor.set(pidController.calculate(getWristDegrees()));

        logDebugInfo();
    }


    public void setState(WristStates state) {
        this.state = state;
    }

    public void setPercentOutput(double percent) {
        wristMotor.set(percent);
    }

    public WristStates getState() {
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
        return angle;
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
    }

}

