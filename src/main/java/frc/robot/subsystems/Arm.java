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


public class Arm extends SubsystemBase{

    public ArmState state = ArmState.HOME;
    public enum ArmState {
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
        // pidController.setMinMax(-Constants.Arm.kMaxSpeed, Constants.Arm.kMaxSpeed);

        setEncoderHomeOffset();
    }


    @Override
    public void periodic(){
        switch (state) {
            // no manual/off yet :(
            case HOME:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmHOME);
                break;
            case LOW_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmLOW_CONE);
                break;
            case MID_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmMID_CONE);
                break;
            case HIGH_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmHIGH_CONE);
                break;
            case LOW_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmLOW_CUBE);
                break;
            case MID_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmMID_CUBE);
                break;
            case HIGH_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmHIGH_CUBE);
                break;
            case INTAKE_CUBE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmINTAKE_CUBE);
                break;
            case INTAKE_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmINTAKE_CONE);
                break;
            case INTAKE_FLOOR_CONE:
                pidController.setSetpoint(Constants.SuperstructureSetpoints.kArmINTAKE_FLOOR_CONE);
                break;
            
            case MANUAL:
                // idk
                break;
            case OFF:
                // idk
                break;
        }

        // if (getIsInBounds() && state != ArmState.OFF)
            // armLeft.set(pidController.calculate(getArmDegrees()));

        logDebugInfo();
    }


    public void setState(ArmState state) {
        this.state = state;
    }

    public void setPercentOutput(double percent) {
        armLeft.set(percent);
    }

    public ArmState getState() {
        return state;
    }

    public boolean reachedStatePos() {
        return Math.abs(getArmDegrees() - pidController.getSetpoint()) < Constants.Arm.reachedStatePosTolerance;
    }

    public void setEncoderHomeOffset() {
        armLeftEncoder.setPosition(Constants.Arm.homeOffset);
    }

    /**
     * @return Current height of arm in meters
     */
    public double getArmDegrees() {
        double angle = Conversions.RotationsToDegrees(armLeftEncoder.getPosition(), Constants.Arm.kArmGearRatio);
        return 90 - (angle);
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
        SmartDashboard.putString("Arm Angle (degrees)", "" + getArmDegrees());
    }

}
