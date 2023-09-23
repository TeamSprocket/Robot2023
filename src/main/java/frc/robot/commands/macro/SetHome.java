package frc.robot.commands.macro;

import frc.util.commands.MacroCommand;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.*;
import frc.robot.Constants;
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
        
        if (timer.get() > 0 && timer.get() < 0.8){
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.setArmAngleSpeed(arm.getArmAngle(), -15, 0.2);
            elevator.setElevatorPositionSpeed(0, -15, 0.6);
        }
        else if (timer.get() > 0.7 && timer.get() < 1.05){
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.moveArm(0.325);
            elevator.setElevatorPositionSpeed(0, -15, 0.5);
        }
        else if (timer.get() > 1.05 && timer.get() < 1.2){
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.moveArm(0.225);
            elevator.setElevatorPositionSpeed(0, -15, 0.4);
        }
        else if (timer.get() > 1.2 && timer.get() < 1.5){
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.moveArm(0.175);
            elevator.setElevatorPositionSpeed(0, -15, 0.2);
        }
        else{
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.moveArm(0.125);
            elevator.setElevatorPositionSpeed(0, 0, 0.1);
        }

        // if (timer.get() > Constants.Elevator.kSetHomeTimeTolerance) {
            // Constants.Drivetrain.CAN_DIRECTION_SWITCH = true;
        // }

        

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
