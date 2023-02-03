package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Arm extends SubsystemBase {
    private final CANSparkMax armMotor_top = new CANSparkMax(0,MotorType.kBrushless);
    private final CANSparkMax armMotor_bottom = new CANSparkMax(0, MotorType.kBrushless);
    
    public Arm(){
        armMotor_top.setInverted(false);
        armMotor_bottom.setInverted(false);
    }

    public void setOutput(double output) {
        armMotor_top.set(output);
        armMotor_bottom.set(output);
    }

    public void stop() {
        armMotor_top.stopMotor();
        armMotor_bottom.stopMotor();
    }


}
