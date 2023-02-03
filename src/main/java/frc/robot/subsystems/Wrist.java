package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Wrist extends SubsystemBase {
    private final CANSparkMax wristMotor = new CANSparkMax(RobotMap.Wrist.WRIST_MOTOR, MotorType.kBrushless);
    

    public Wrist() {
        wristMotor.setInverted(false);
    }

    public void setOutput(double output) {
        wristMotor.set(output);
    }
 
    public void stop() {
        wristMotor.stopMotor();
    }
}