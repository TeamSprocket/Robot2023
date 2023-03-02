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

public class SetHigh extends MacroCommand{
    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer timer;
    
    private double startTime;

    public SetHigh (Elevator elevator, Arm arm, Wrist wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        timer = new Timer();

        addRequirements(elevator, arm, wrist);
        
    }


    public void initialize(){
        wrist.moveWrist(-0.1115);
        startTime = System.currentTimeMillis();
        timer.reset();
    }

    @Override
    public void execute() {
      //new SequentialCommandGroup(
        timer.start();
        
        if (timer.get() > 0.1 && timer.get() < 1){
            elevator.setElevatorPosition(elevator.getElevatorHeight(), -12.13);
        }
        else if(timer.get()> 1 && timer.get() < 2){
            arm.setArmAngle(arm.getArmAngle(), -80);
        }
        else{
            elevator.setElevatorPosition(elevator.getElevatorHeight(), -12.13);
            arm.setArmAngle(arm.getArmAngle(), -80);
        }
        
        
        // new ElevatePosition(elevator, -13.13) //elevator up
        // new ParallelCommandGroup(
        //     new ElevatePosition(elevator, -13.13), //keep elevator up
        //     new MoveArmPosition(arm, -80) //arm up
        // //new MoveWristAngle(wrist, -250) // move wrist
        // ),
        // new ParallelCommandGroup(
        // new ElevatePosition(elevator, 34.45866374), //elevator down
        // new MoveArmPosition(arm, -80) // keep arm up
        // )

        
      
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
        elevator.setElevatorPosition(elevator.getElevatorHeight(), -13.13);
        arm.setArmAngle(arm.getArmAngle(), -80);
    }
}
