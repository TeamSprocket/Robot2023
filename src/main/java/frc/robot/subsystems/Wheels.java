package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Wheels extends SubsystemBase {

    private final CANSparkMax wheelMotorLeft = new CANSparkMax(RobotMap.Intake.INTAKE, MotorType.kBrushless);
    private final CANSparkMax wheelMotorRight = new CANSparkMax(RobotMap.Intake.INTAKE, MotorType.kBrushless);

    public Wheels() {
        wheelMotorLeft.setInverted(false);
        wheelMotorRight.setInverted(true);
    }

    public void setOutput(double output) {
        wheelMotorLeft.set(output);
        wheelMotorRight.set(output);
    }
        
    public void stop() {
        wheelMotorLeft.stopMotor();
        wheelMotorRight.stopMotor();
    }
}