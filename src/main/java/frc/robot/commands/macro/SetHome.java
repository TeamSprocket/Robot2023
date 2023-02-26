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

public class SetHome extends MacroCommand{
    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer timer;
    
    private double startTime;

    public SetHome(Elevator elevator, Arm arm, Wrist wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        timer = new Timer();

        addRequirements(elevator, arm, wrist);
        
    }


    public void initialize(){
        wrist.moveWrist(-0.2);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
      //new SequentialCommandGroup(
        timer.start();
        
        if (timer.get() > 0 && timer.get() < 0.75){
            elevator.setElevatorPosition(elevator.getElevatorHeight(), -20);
            arm.setArmAngleSlow(arm.getArmAngle(), -35);
        }
        if (timer.get() > 0.75 && timer.get() < 3.75){
            arm.setArmAngleSlow(arm.getArmAngle(), 1);
        }
        if (timer.get() > 3.75 && timer.get() < 4){
            elevator.setElevatorPosition(elevator.getElevatorHeight(), 32);
        }
        if (timer.get() > 4.15){
            timer.reset();
        }
        
        
      
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
        elevator.setElevatorPosition(elevator.getElevatorHeight(), 32);
        arm.setArmAngle(arm.getArmAngle(), 0);
    }
}
