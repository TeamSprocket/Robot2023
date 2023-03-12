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

public class DeportArm extends MacroCommand{
    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer timer;
    
    private double startTime;

    public DeportArm (Elevator elevator, Arm arm, Wrist wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
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
        
    }

    @Override
    public boolean isFinished(){
        if (timer.get() >= 2) {
            return true;
          }
          return false;
    }

    @Override
    public void end(boolean interrupted){
        wrist.setWristAngle(wrist.getWristAngle(), 0);
        arm.setArmAngleSpeed(arm.getArmAngle(), 0, 0.1);
        elevator.setElevatorPositionSpeed(0, -15, 0.6);
    }
}
