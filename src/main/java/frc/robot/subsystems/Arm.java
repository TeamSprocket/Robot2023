package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
    private final CANSparkMax armMotor_Left = new CANSparkMax(RobotMap.Arm.ARM_LEFT,MotorType.kBrushless);
    private final CANSparkMax armMotor_Right = new CANSparkMax(RobotMap.Arm.ARM_RIGHT, MotorType.kBrushless);
    
    public Arm(){
        armMotor_Left.setInverted(false);
        armMotor_Right.setInverted(false);
    }

    public void setOutput(double output) {
        armMotor_Left.set(output);
        armMotor_Right.set(output);
    }

    public void stop() {
        armMotor_Left.stopMotor();
        armMotor_Right.stopMotor();
    }


}
