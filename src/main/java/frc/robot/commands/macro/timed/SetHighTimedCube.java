package frc.robot.commands.macro.timed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.macro.*;
import frc.robot.commands.macro.ElevatePosition;
// import frc.robot.commands.macro.ElevatePositionTimed;
import frc.robot.commands.macro.MoveArmPosition;
import frc.robot.subsystems.*;
import frc.util.commands.MacroCommand;

public class SetHighTimedCube extends MacroCommand{
    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer timer;
    private final double duration;
    
    private double startTime;

    public SetHighTimedCube (Elevator elevator, Arm arm, Wrist wrist, double duration) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        this.duration = duration;

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
            wrist.setWristAngle(wrist.getWristAngle(), -22.5);
            arm.setArmAngleSpeed(arm.getArmAngle(), -80, 0.4);
            elevator.setElevatorPositionSpeed(0, 5, 0.6);
        }
        else{
            wrist.setWristAngle(wrist.getWristAngle(), -22.5);
            arm.setArmAngleSpeed(arm.getArmAngle(), -80, 0.2);
            elevator.setElevatorPositionSpeed(0, 5, 0.2);
        }
        
        
    }

    public boolean isFinished(){
        if (timer.get() >= duration) {
            return true;
          }
          return false;
    }

    @Override
    public void end(boolean interrupted){
        wrist.setWristAngle(wrist.getWristAngle(), -22.5);
        arm.setArmAngleSpeed(arm.getArmAngle(), -80, 0.2);
        elevator.setElevatorPositionSpeed(0, 5, 0.2);
    }
}
