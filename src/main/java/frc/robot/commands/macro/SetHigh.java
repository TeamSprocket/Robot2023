package frc.robot.commands.macro;

import frc.util.commands.MacroCommand;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.macro.*;

public class SetHigh extends MacroCommand{
    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer timer;
    
    public SetHigh (Elevator elevator, Arm arm, Wrist wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;

        timer = new Timer();

        addRequirements(elevator, arm, wrist);

        // Constants.Drivetrain.CAN_DIRECTION_SWITCH = false;
        
    }


    public void initialize(){
        timer.reset();
    }

    @Override
    public void execute() {
        timer.start();
        
        if (timer.get() > 0.01 && timer.get() < 1.75){
            wrist.setWristAngle(wrist.getWristAngle(), 17.5);
            arm.setArmAngleSpeed(arm.getArmAngle(), -78, 0.5);
            elevator.setElevatorPositionSpeed(0, -15, 0.6);
        }
        else{
            wrist.setWristAngle(wrist.getWristAngle(), 17.5);
            arm.setArmAngleSpeed(arm.getArmAngle(), -78, 0.35);
            elevator.setElevatorPositionSpeed(0, -15, 0.3);
        }
        
        
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
        wrist.setWristAngle(wrist.getWristAngle(), 17.5);
        arm.setArmAngleSpeed(arm.getArmAngle(), -78, 0.25);
        elevator.setElevatorPositionSpeed(0, -15, 0.3);
    }
}
