package frc.robot.commands.macro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.macro.*;
import frc.robot.commands.macro.ElevatePosition;
// import frc.robot.commands.macro.ElevatePositionTimed;
import frc.robot.commands.macro.MoveArmPosition;
import frc.robot.subsystems.*;
import frc.util.commands.MacroCommand;

public class SetLow extends MacroCommand{
    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer timer;
    
    private double startTime;

    public SetLow (Elevator elevator, Arm arm, Wrist wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        timer = new Timer();

        addRequirements(elevator, arm, wrist);
        
    }


    public void initialize(){
        wrist.moveWrist(-0.11025);
        startTime = System.currentTimeMillis();
        timer.reset();
    }

    @Override
    public void execute() {
      //new SequentialCommandGroup(
        timer.start();
        
        if (timer.get() > 0.1 && timer.get() < 0.75){
            elevator.setElevatorPosition(elevator.getElevatorHeight(), -13.13);
            arm.setArmAngle(arm.getArmAngle(), -30);
        }
        else if (timer.get() > 0.75 && timer.get() < 1.5){
            arm.setArmAngle(arm.getArmAngle(), -25);
        }
        else if(timer.get() > 1.5 && timer.get() < 1.75){
            elevator.setElevatorPosition(elevator.getElevatorHeight(), 30);
            arm.setArmAngle(arm.getArmAngle(), -25);
        }else{
            elevator.setElevatorPosition(elevator.getElevatorHeight(), 30);
            arm.setArmAngle(arm.getArmAngle(), -25);
        }
      
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
        elevator.setElevatorPosition(elevator.getElevatorHeight(), 30);
        arm.setArmAngle(arm.getArmAngle(), -24.5);
    }
}
