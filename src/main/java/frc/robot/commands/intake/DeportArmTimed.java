package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.util.commands.MacroCommand;

public class DeportArmTimed extends MacroCommand{
    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final double duration;
    private final Timer timer;
    
    // private double startTime;

    public DeportArmTimed (Elevator elevator, Arm arm, Wrist wrist, double duration) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        this.duration = duration;

        timer = new Timer();

        addRequirements(elevator, arm, wrist);
        
    }

    @Override
    public void initialize(){
        timer.reset();
    }

    @Override
    public void execute() {
        timer.start();
        
        if (timer.get() > 0 && timer.get() < 0.5){
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.setArmAngleSpeed(arm.getArmAngle(), 0, 0.1);
            elevator.setElevatorPositionSpeed(0, -15, 0.6);
        }
        else if (timer.get() > 0.5 && timer.get () > 1){
            wrist.setWristAngle(wrist.getWristAngle(), 5);
            arm.setArmAngleSpeed(arm.getArmAngle(), -20, 0.1);
        }
        else{
            wrist.setWristAngle(wrist.getWristAngle(), 5);
            arm.setArmAngleSpeed(arm.getArmAngle(), -20, 0.1);
            elevator.setElevatorPositionSpeed(0, -5, 0.6);
        }
        
    }

    @Override
    public boolean isFinished(){
        if (timer.get() >= duration) {
            return true;
          }
          return false;
    }

    @Override
    public void end(boolean interrupted){
        wrist.setWristAngle(wrist.getWristAngle(), 5);
        arm.setArmAngleSpeed(arm.getArmAngle(), -20, 0.1);
        elevator.setElevatorPositionSpeed(0, -5, 0.6);
    }
}
