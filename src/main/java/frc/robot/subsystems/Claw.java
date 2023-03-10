package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants;


public class Claw extends SubsystemBase{

    private final WPI_TalonFX claw = new WPI_TalonFX(RobotMap.Claw.CLAW);

    private final DoubleSolenoid clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.Claw.PISTON_LEFT, RobotMap.Claw.PISTON_RIGHT);
   
    public Claw() {
        claw.setInverted(false);

        
        claw.setNeutralMode(NeutralMode.Brake);

    }

    public void moveClaw(double output){
        //claw.set(output * 0.2);

        claw.set(ControlMode.PercentOutput, output);
        //percent output ouputs as a percentange from -1 to 1 with 0 stopping the motor
        //other modes are current mode, where the output is in amps,
        //position mode, where the output is in encoder ticks or analog values
        //and follower mode, where the output is the interger device ID (idk what this is)
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

    public double getVelocity() {
        return claw.getSelectedSensorPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Claw] RPM", getVelocity());
    }
}
