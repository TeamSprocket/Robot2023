package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Claw extends SubsystemBase{
    private final DoubleSolenoid clawPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.Intake.PISTON_LEFT_FORWARD, RobotMap.Intake.PISTON_RIGHT_FORWARD);
   
    public void actuateIntakeArm(boolean out) {
        if (out){
            clawPiston.set(Value.kForward);
        }
        else{
            clawPiston.set(Value.kReverse);
        }
    }

    public boolean isActuated() {   
        if(clawPiston.get()==Value.kReverse) {
            return true;
        }
        return false;
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("[Intake] isActuated", isActuated());
    }
}
