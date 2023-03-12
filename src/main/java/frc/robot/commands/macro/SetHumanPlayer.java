package frc.robot.commands.macro;

import frc.util.commands.MacroCommand;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.*;
import frc.robot.commands.macro.*;

public class SetHumanPlayer extends MacroCommand{
    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer timer;
    
    public SetHumanPlayer (Elevator elevator, Arm arm, Wrist wrist) {
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

        if (timer.get() > 0 && timer.get() < 2){
            wrist.setWristAngle(wrist.getWristAngle(), 10);
            arm.setArmAngleSpeed(arm.getArmAngle(), -80, 0.35);
            elevator.setElevatorPositionSpeed(0, -13, 0.6);
        }
        else{
            wrist.setWristAngle(wrist.getWristAngle(), 10);
            arm.setArmAngleSpeed(arm.getArmAngle(), -80, 0.35);
            elevator.setElevatorPositionSpeed(0, -13, 0.15);
        }
        
        // if (elevator.getElevatorHeight() < 3 && elevator.getElevatorHeight() > -3){
        //     if (timer.get() > 0 && timer.get() < 1){
        //         wrist.setWristAngle(wrist.getWristAngle(), 0);
        //         arm.setArmAngleSpeed(arm.getArmAngle(), 0, 0.1);
        //         elevator.setElevatorPositionSpeed(0, -15, 0.6);
        //     }
        //     else if (timer.get() > 1 && timer.get() < 2){
        //         wrist.setWristAngle(wrist.getWristAngle(), 10);
        //         arm.setArmAngleSpeed(arm.getArmAngle(), -80, 0.35);
        //         elevator.setElevatorPositionSpeed(0, -13, 0.1);
        //     }
        // }
        // else{
        //     if (timer.get() > 0 && timer.get() < 2){
        //         wrist.setWristAngle(wrist.getWristAngle(), 10);
        //         arm.setArmAngleSpeed(arm.getArmAngle(), -80, 0.35);
        //         elevator.setElevatorPositionSpeed(0, -13, 0.3);
        //     }
        // }
        
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
        wrist.setWristAngle(wrist.getWristAngle(), 10);
        arm.setArmAngleSpeed(arm.getArmAngle(), -80, 0.35);
        elevator.setElevatorPositionSpeed(0, -13, 0.1);
    }
}
