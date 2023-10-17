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

public class SetHomeTimed extends MacroCommand{
    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer timer;
    private final double duration;
    
    private double startTime;

    public SetHomeTimed(Elevator elevator, Arm arm, Wrist wrist, double duration) {
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
        
        if (timer.get() > 0 && timer.get() < 0.6){
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.setArmAngleSpeed(arm.getArmAngle(), -15, 0.2);
            elevator.setElevatorPositionSpeed(0, -15, 0.6);
        }
        else if (timer.get() > 0.6 && timer.get() < 0.85){
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.moveArm(0.3);
            elevator.setElevatorPositionSpeed(0, -15, 0.5);
        }
        else if (timer.get() > 0.85 && timer.get() < 1){
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.moveArm(0.15);
            elevator.setElevatorPositionSpeed(0, -15, 0.4);
        }
        else if (timer.get() > 1 && timer.get() < 1.3){
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.moveArm(0.1);
            elevator.setElevatorPositionSpeed(0, -15, 0.2);
        }
        else{
            wrist.setWristAngle(wrist.getWristAngle(), 0);
            arm.moveArm(0.05);
            elevator.setElevatorPositionSpeed(0, 0, 0.1);
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
        wrist.setWristAngle(wrist.getWristAngle(), 0);
        arm.moveArm(0.05);
        elevator.setElevatorPositionSpeed(0, 0, 0.1);
    }
}
