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


public class Arm extends SubsystemBase{

    public ArmStates state = ArmStates.HOME;
    public enum ArmStates {
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
    
    private CANSparkMax armLeft = new CANSparkMax(RobotMap.Arm.ARM_LEFT, MotorType.kBrushless);
    private CANSparkMax armRight = new CANSparkMax(RobotMap.Arm.ARM_RIGHT, MotorType.kBrushless);

    private RelativeEncoder armLeftEncoder = armLeft.getEncoder();
    private RelativeEncoder armRightEncoder = armRight.getEncoder();

    private final PIDPlus pidController = new PIDPlus(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD);
    

    public Arm() {
        armLeft.restoreFactoryDefaults();
        armRight.restoreFactoryDefaults();
        
        armLeft.setInverted(false);
        armRight.setInverted(false);

        armRight.follow(armLeft);

        pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmHOME);
        pidController.setMinMax(-Constants.Arm.kMaxSpeed, Constants.Arm.kMaxSpeed);
    }


    @Override
    public void periodic(){
        switch (state) {
            // no manual/off yet :(
            case HOME:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmHOME);
            case LOW_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmLOW_CONE);
            case MID_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmMID_CONE);
            case HIGH_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmHIGH_CONE);
            case LOW_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmLOW_CUBE);
            case MID_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmMID_CUBE);
            case HIGH_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmHIGH_CUBE);
            case IN_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmIN_CUBE);
            case IN_CONE_STANDING:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmIN_CONE_STANDING);
            case IN_CONE_FLOOR:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmIN_CONE_FLOOR);
            
            case MANUAL:
                // idk
            case OFF:
                // idk
        }

        if (getIsInBounds() && state != ArmStates.OFF)
            armLeft.set(pidController.calculate(getArmDegrees()));

        logDebugInfo();
    }


    public void setState(ArmStates state) {
        this.state = state;
    }

    public void setPercentOutput(double percent) {
        armLeft.set(percent);
    }

    public ArmStates getState() {
        return state;
    }

    public boolean reachedStatePos() {
        return Math.abs(getArmDegrees() - pidController.getSetpoint()) < Constants.Arm.reachedStatePosTolerance;
    }

    /**
     * @return Current height of arm in meters
     */
    public double getArmDegrees() {
        double angle = Conversions.RotationsToDegrees(armLeftEncoder.getPosition(), Constants.Arm.kArmGearRatio);
        return angle;
    }

    public boolean getIsInBounds() {
        return (getArmDegrees() >= Constants.Arm.MIN_ANGLE && getArmDegrees() <= Constants.Arm.MAX_ANGLE);
    }

    public void resetEncoders() {
        armLeft.getEncoder().setPosition(0);
        armRight.getEncoder().setPosition(0);
    }

    public void clearStickyFaults() {
        armLeft.clearFaults();
        armRight.clearFaults();
    }
    
    public void stop() {
        armLeft.stopMotor();
        armRight.stopMotor();
    }

    public void logDebugInfo() {
        SmartDashboard.putString("Arm State", state.toString());
    }

}
