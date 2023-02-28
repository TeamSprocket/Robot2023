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
        timer.reset();
    }

    @Override
    public void execute() {
      //new SequentialCommandGroup(
        timer.start();
        
        if (timer.get() > 0.1 && timer.get() < 1.55){
            elevator.setElevatorPosition(elevator.getElevatorHeight(), -20);
            arm.setArmAngle(arm.getArmAngle(), -35);
        }
        else if (timer.get() > 1.55 && timer.get() < 4.55){
            elevator.setElevatorPosition(elevator.getElevatorHeight(), -20);
            arm.setArmAngle(arm.getArmAngle(), 0.3);
        }
        else if (timer.get() > 4.55 && timer.get() < 4.8){
            elevator.setElevatorPosition(elevator.getElevatorHeight(), 28);
            arm.setArmAngle(arm.getArmAngle(), 0.3);
            
        }
        else{
            // elevator.setElevatorPosition(elevator.getElevatorHeight(), 28);
            arm.setArmAngle(arm.getArmAngle(), 0.3);
            if (Math.abs(arm.getArmAngle()) < 0.5){
                arm.moveArm(0);
            }
        }
        
        
      
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
        elevator.setElevatorPosition(elevator.getElevatorHeight(), 28);
        arm.setArmAngle(arm.getArmAngle(), 0.3);
    }
}
