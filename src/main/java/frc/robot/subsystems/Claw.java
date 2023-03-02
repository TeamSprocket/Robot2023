package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants;


public class Claw extends SubsystemBase{

    private final WPI_TalonFX leftClaw = new WPI_TalonFX(RobotMap.Claw.CLAW_LEFT);
    private final WPI_TalonFX rightClaw = new WPI_TalonFX(RobotMap.Claw.CLAW_RIGHT);

    private final DoubleSolenoid clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.Claw.PISTON_LEFT, RobotMap.Claw.PISTON_RIGHT);
   
    public Claw() {
        leftClaw.setInverted(false);
        rightClaw.setInverted(true);
        
        leftClaw.setNeutralMode(NeutralMode.Coast);
        rightClaw.setNeutralMode(NeutralMode.Coast);

        rightClaw.follow(leftClaw);
    }

    public void moveClaw(double output){
        leftClaw.set(output);
        rightClaw.set(output);
        // rightClaw.set(ControlMode.PercentOutput, output);
    }

    public void actuateClaw(boolean out) {
        if (out){
            clawSolenoid.set(Value.kForward);
        }
        else{
            clawSolenoid.set(Value.kReverse);
        }
    }

    public boolean isActuated() {   
        if(clawSolenoid.get() == Value.kReverse) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("[Intake] isActuated", isActuated());
    }
}