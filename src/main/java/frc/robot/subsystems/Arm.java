package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class Arm extends SubsystemBase {
    private final CANSparkMax armMotor_top = new CANSparkMax(0,MotorType.kBrushless);
    private final CANSparkMax armMotor_bottom = new CANSparkMax(0, MotorType.kBrushless);
    
    public arm(){
        armMotor_top.setInverted(false);
        armMotor_bottom.setInverted(false);
    }

    
}
