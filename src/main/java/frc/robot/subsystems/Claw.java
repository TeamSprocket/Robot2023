package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Claw extends SubsystemBase{
    private final DoubleSolenoid intakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.Intake.PISTON_FORWARD, RobotMap.Intake.PISTON_REVERSE);
    public void actuateIntakeArm(boolean isActuated) {
        if (isActuated){
            intakePiston.set(Value.kForward);
        }
        else{
            intakePiston.set(Value.kReverse);
        }
    }
    public boolean isActuated() {   
        if(intakePiston.get()==Value.kReverse) {
            return true;
        }
        return false;
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("[Intake] isActuated", isActuated());
    }
}
