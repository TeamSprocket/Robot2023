package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.util.ShuffleboardPIDTuner;
import frc.robot.Constants;

public class Intake extends SubsystemBase{

    private final WPI_TalonFX intake = new WPI_TalonFX(RobotMap.Intake.CLAW);
    double idleSpeed = 0;
    double activeSpeed = 0;

    
    public Intake() {
        intake.setInverted(false);
        intake.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,50,50,1.0));
        intake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,50,50,1.0));

        ShuffleboardPIDTuner.addSlider("kIdleSpeed", 0, 1, 0.1);

        intake.setNeutralMode(NeutralMode.Brake);

    }

    public void moveIntake(double output){
        if (output > 0) {
            idleSpeed = ShuffleboardPIDTuner.get("kIdleSpeed");
        } else if (output < 0) {
            idleSpeed = -ShuffleboardPIDTuner.get("kIdleSpeed");
        }


        activeSpeed = idleSpeed + output;
        
        //percent output ouputs as a percentange from -1 to 1 with 0 stopping the motor
        //other modes are current mode, where the output is in amps,
        //position mode, where the output is in encoder ticks or analog values
        //and follower mode, where the output is the interger device ID (idk what this is)
    }

    public void clearStickyFaults() {
        intake.clearStickyFaults();
    }



    public double getVelocity() {
        return intake.getSelectedSensorVelocity();
    }

    @Override
    public void periodic() {
        intake.set(ControlMode.PercentOutput, activeSpeed);
        SmartDashboard.putNumber("[Intake] RPM", getVelocity());
    }
}
