package frc.robot.commands.intake;

import frc.util.commands.MacroCommand;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.*;


public class SetMid extends MacroCommand{
    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer timer;
    
    public SetMid (Elevator elevator, Arm arm, Wrist wrist) {
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
            wrist.setWristAngle(wrist.getWristAngle(), 17.5);
            arm.setArmAngleSpeed(arm.getArmAngle(), -60, 0.4);
            elevator.setElevatorPositionSpeed(0, -13, 0.55);
        }
        else{
            wrist.setWristAngle(wrist.getWristAngle(), 17.5);
            arm.setArmAngleSpeed(arm.getArmAngle(), -60, 0.2);
            elevator.setElevatorPositionSpeed(0, -13, 0.15);
        }
        
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
            wrist.setWristAngle(wrist.getWristAngle(), 17.5);
            arm.setArmAngleSpeed(arm.getArmAngle(), -60, 0.2);
            elevator.setElevatorPositionSpeed(0, -13, 0.1);
    }
}
