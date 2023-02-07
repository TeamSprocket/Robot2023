package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Wheels extends SubsystemBase {

    private final WPI_TalonFX wheelMotorLeft = new WPI_TalonFX(RobotMap.Intake.INTAKE);
    private final WPI_TalonFX wheelMotorRight = new WPI_TalonFX(RobotMap.Intake.INTAKE);

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