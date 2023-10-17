package frc.robot.commands.superstructure.auton;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.macro.ElevatePositionTimed;
import frc.robot.commands.macro.*;
import frc.robot.commands.superstructure.ElevatePosition;
import frc.robot.commands.superstructure.MoveArmPosition;
import frc.robot.subsystems.*;
import frc.util.wpilib_defaults.commands.MacroCommand;

public class SetLowCubeTimed extends MacroCommand{
    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer timer;
    private final double duration;
    
    private double startTime;

    public SetLowCubeTimed (Elevator elevator, Arm arm, Wrist wrist, double duration) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        this.duration = duration;
        timer = new Timer();

        addRequirements(elevator, arm, wrist);
        
    }


    public void initialize(){
        timer.reset();
    }

    @Override
    public void execute() {
        timer.start();
        
        if (timer.get() > 0.1 && timer.get() < 0.75){
            wrist.setWristAngle(wrist.getWristAngle(), 2.5);
            arm.setArmAngle(arm.getArmAngle(), -22.5);
            elevator.setElevatorPositionSpeed(elevator.getElevatorHeight(), 30, 0.45);
        }
        else{
            wrist.setWristAngle(wrist.getWristAngle(), 2.5);
            arm.setArmAngle(arm.getArmAngle(), -22.5);
            elevator.setElevatorPositionSpeed(elevator.getElevatorHeight(), 30, 0.2);
        }
        
    }
    public boolean isFinished(){
        if (timer.get() >= duration) {
            return true;
          }
          return false;
    }

    @Override
    public void end(boolean interrupted){
        elevator.setElevatorPosition(elevator.getElevatorHeight(), 30);
        wrist.setWristAngle(wrist.getWristAngle(), 2.5);
        arm.setArmAngle(arm.getArmAngle(), -22.5);
        elevator.setElevatorPositionSpeed(elevator.getElevatorHeight(), 30, 0.2);
    }
}
