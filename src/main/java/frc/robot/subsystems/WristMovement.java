package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class WristMovement extends SubsystemBase {
    private final CANSparkMax wristMotor = new CANSparkMax(RobotMap.Shooter.SHOOTER_SPARK_MAX, MotorType.kBrushless);
    

    public WristMovement() {
        wristMotor.setInverted(false);
    }

    public void setOutput(double output) {
        wristMotor.set(output);
    }
 
    public void stop() {
        wristMotor.stopMotor();
    }
}