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
        wrist.moveWrist(-0.1015);
        timer.reset();
    }

    @Override
    public void execute() {
      //new SequentialCommandGroup(
        timer.start();
        
        if (timer.get() > 0.1 && timer.get() < 1){
            elevator.setElevatorPosition(elevator.getElevatorHeight(), -9.5);
        }
        else if(timer.get()> 1 && timer.get() < 2){
            arm.setArmAngle(arm.getArmAngle(), -70);
        }
        else{
            elevator.setElevatorPosition(elevator.getElevatorHeight(), -9.5);
            arm.setArmAngle(arm.getArmAngle(), -70);
        }
        
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
        elevator.setElevatorPosition(elevator.getElevatorHeight(), -9.5);
        arm.setArmAngle(arm.getArmAngle(), -70);
    }
}
