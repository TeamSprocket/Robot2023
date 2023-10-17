package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.*;
import frc.util.wpilib_defaults.commands.MacroCommand;
import frc.robot.Constants;
import frc.robot.commands.macro.*;

public class SetLowConeTilted extends MacroCommand{
    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer timer;
    
    private double startTime;

    public SetLowConeTilted (Elevator elevator, Arm arm, Wrist wrist) {
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
        
        if (timer.get() > 0.1 && timer.get() < 0.5){
            wrist.setWristAngle(wrist.getWristAngle(), 25.5);
            arm.setArmAngle(arm.getArmAngle(), -24);
            elevator.setElevatorPositionSpeed(elevator.getElevatorHeight(), 30, 0.6);
        }
        else{
            wrist.setWristAngle(wrist.getWristAngle(), 25.5);
            arm.setArmAngle(arm.getArmAngle(), -24);
            elevator.setElevatorPositionSpeed(elevator.getElevatorHeight(), 30, 0.2);
        }
        
      
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
        wrist.setWristAngle(wrist.getWristAngle(), 25.5);
        arm.setArmAngle(arm.getArmAngle(), -24);
        elevator.setElevatorPositionSpeed(elevator.getElevatorHeight(), 30, 0.2);
    }
}