package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import frc.robot.RobotMap;

public class Wheels extends SubsystemBase {

    private final WPI_TalonFX clawLeft = new WPI_TalonFX(RobotMap.Intake.INTAKE);
    private final WPI_TalonFX clawRight = new WPI_TalonFX(RobotMap.Intake.INTAKE);

    public Wheels() {
        clawLeft.setInverted(false); //adjust and change later
        clawRight.setInverted(true);

        clawLeft.setNeutralMode(NeutralMode.Coast);
        clawRight.setNeutralMode(NeutralMode.Coast);

        clawLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 00));
        clawRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 00)); 
        
        //clawLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 45, 00)); 
        //clawRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 45, 00));

        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.voltageCompSaturation = 10; //change
        clawLeft.configAllSettings(leftConfig);

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.voltageCompSaturation = 10; //change
        clawRight.configAllSettings(rightConfig);
    }

    public void setSpeed(double speed) {
        clawLeft.set(speed);
        clawRight.set(speed);
    }
        
    public void stop() {
        clawLeft.stopMotor();
        clawRight.stopMotor();
    }
}