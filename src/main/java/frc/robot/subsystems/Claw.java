package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants;

public class Claw extends SubsystemBase{

    private final WPI_TalonFX claw = new WPI_TalonFX(RobotMap.Claw.CLAW);

    
    public Claw() {
        claw.setInverted(false);
        claw.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,50,50,1.0));
        claw.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,50,50,1.0));



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


    public double getVelocity() {
        return claw.getSelectedSensorVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Claw] RPM", getVelocity());
    }
}
