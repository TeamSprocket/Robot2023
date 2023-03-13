package frc.robot.commands.macro;

import frc.util.commands.MacroCommand;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.*;
import frc.robot.commands.macro.*;

public class SetHome extends MacroCommand{
    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer timer;
    
    public SetHome(Elevator elevator, Arm arm, Wrist wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        timer = new Timer();

        addRequirements(elevator, arm, wrist); 
    }

    public void initialize(){
        timer.reset();
    }

    @Override
    public void execute() {
        timer.start();
        
        if (timer.get() > 0 && timer.get() < 0.6){
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.setArmAngleSpeed(arm.getArmAngle(), -15, 0.2);
            elevator.setElevatorPositionSpeed(0, -15, 0.6);
        }
        else if (timer.get() > 0.6 && timer.get() < 0.85){
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.moveArm(0.3);
            elevator.setElevatorPositionSpeed(0, -15, 0.5);
        }
        else if (timer.get() > 0.85 && timer.get() < 1){
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.moveArm(0.15);
            elevator.setElevatorPositionSpeed(0, -15, 0.4);
        }
        else if (timer.get() > 1 && timer.get() < 1.3){
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.moveArm(0.1);
            elevator.setElevatorPositionSpeed(0, -15, 0.2);
        }
        else{
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.moveArm(0.05);
            elevator.setElevatorPositionSpeed(0, 0, 0.1);
        }

    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
        wrist.setWristAngle(wrist.getWristAngle(), 0);
        arm.setArmAngleSpeed(arm.getArmAngle(), 0, 0.15);
        elevator.setElevatorPositionSpeed(0, 0, 0.2);
    }
}
