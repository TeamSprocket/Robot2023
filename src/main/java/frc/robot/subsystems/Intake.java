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

    private final WPI_TalonFX intakemotor = new WPI_TalonFX(RobotMap.Claw.CLAW);
    double idleSpeed = 0;
    double activeConeSpeed = 0;
    double activeCubeSpeed = 0;
    
    private boolean containCone = false;
    private boolean containCube = false;

    private int direction = 1;

    private double intakeconeSpeed = 0.8;
    private double intakecubeSpeed = -0.5; // one direction is for cone, other is for cube
    
    private double outtakeconeSpeed = -0.8;
    private double outtakecubeSpeed = 0.5;

    private double maxvoltage = 0.0;

    
    public Intake() {
        intakemotor.setInverted(false);
        intakemotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,50,50,1.0));
        intakemotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,50,50,1.0));

        ShuffleboardPIDTuner.addSlider("kIdleSpeed", 0, 1, 0.1);

        intakemotor.setNeutralMode(NeutralMode.Brake);

    }

    // public void moveClaw(double output){
    //     if (output > 0) {
    //         idleSpeed = ShuffleboardPIDTuner.get("kIdleSpeed");
    //     } else if (output < 0) {
    //         idleSpeed = -ShuffleboardPIDTuner.get("kIdleSpeed");
    //     }


    //     activeSpeed = idleSpeed + output;
        
    //     //percent output ouputs as a percentange from -1 to 1 with 0 stopping the motor
    //     //other modes are current mode, where the output is in amps,
    //     //position mode, where the output is in encoder ticks or analog values
    //     //and follower mode, where the output is the interger device ID (idk what this is)
    // }

    public void intakeIdle() {
        intakemotor.set(direction * idleSpeed);
        // take "direction" of the intakemotor and multiply by idle speed to change direction
      }
    
      public void intakeCone() {
    
        if (containCone == true) { 
          // if the robot has a cone
          direction = -1;
          // make direction negative because we want to outtake the cone
          intakemotor.set(outtakeconeSpeed);
          // outtake cone ^^
        }
        else {
          // if we do not have a cone
          direction = 1;
          // make idle speed positive because we are intaking
          intakemotor.set(intakeconeSpeed);
          // intake cone ^^
    
          if (getCurrent() > maxvoltage) {
            // if we get voltage spike, that means we have got a cone
            containCone = true;
            // set boolean to true
          }
          else {
            containCone = false;
            // if the voltage is normal, then we do not have a cone yet
            // as soon as we get a cone we will outtake instead of intake
          }
        }
      }
    
      public void intakeCube() {
        if (containCube == true) { 
          // if the robot has a cube
          direction = 1;
          // make direction positive because that's how we outtake the cube
          intakemotor.set(outtakecubeSpeed);
          // outtake cube ^^
        }
        else {
          // if we do not have a cube
          direction = -1;
          // make idle speed negative because that is how we intake cube
          intakemotor.set(intakecubeSpeed);
          // intake cube ^^
    
          if (-getCurrent() /* current can be negative based on rotation of intakemotor so be sure to change inquality*/ > maxvoltage) {
            // if we get voltage spike, that means we have got a cube
            containCube = true;
            // set boolean to true
          }
          else {
            containCube = false;
            // if the voltage is normal, then we do not have a cube yet
            // as soon as we get a cube we will outtake instead of intake
          }
        }
      }
    
      public double getCurrent() {
        double current = intakemotor.getStatorCurrent();
        // return current to check if we have an object in the intake
        return current;
      }
    
      public void stopIntake() {
        intakemotor.set(0.0);
      }


    public void clearStickyFaults() {
        intakemotor.clearStickyFaults();
    }

    public double getVelocity() {
        return intakemotor.getSelectedSensorVelocity();
    }

    @Override
    public void periodic() {
        // intakemotor.set(ControlMode.PercentOutput, activeSpeed);
        SmartDashboard.putNumber("[Claw] RPM", getVelocity());
    }
}
