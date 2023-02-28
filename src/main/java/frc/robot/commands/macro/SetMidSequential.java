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

public class SetMidSequential extends MacroCommand{
    
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer timer;

    private boolean homeCheck;
    private boolean elevatorCheck;
    private boolean armCheck;
    
    private double startTime;

    public SetMidSequential (Elevator elevator, Arm arm, Wrist wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        timer = new Timer();

        addRequirements(elevator, arm, wrist);
        
    }


    public void initialize(){
        wrist.moveWrist(-0.115);

        homeCheck = false;
        elevatorCheck = false;
        armCheck = false;

        startTime = System.currentTimeMillis();
        timer.reset();
    }

    @Override
    public void execute() {

        if (elevator.getElevatorHeight() >= -5 || elevator.getElevatorHeight() <= 5){
            elevator.setElevatorPosition(elevator.getElevatorHeight(), -20);
            arm.setArmAngle(arm.getArmAngle(), 0);
            homeCheck = true;
        }
        else if (elevator.getElevatorHeight() < -5){
            elevator.setElevatorPosition(elevator.getElevatorHeight(), -4);
            arm.setArmAngle(arm.getArmAngle(), -4);
        }
        else if (elevator.getElevatorHeight() > 5){
            elevator.setElevatorPosition(elevator.getElevatorHeight(), -4);
            
        }

        // if (homeCheck){
        //     elevator.setElevatorPosition(elevator.getElevatorHeight(), -4);
        //     arm.setArmAngle(arm.getArmAngle(), -60);
        //     elevatorCheck = true;
        // }
        // if (homeCheck == true && elevatorCheck == true){
        //     arm.setArmAngle(arm.getArmAngle(), -60);
        // }


        // timer.start();
        
        // if (timer.get() > 0.1 && timer.get() < 1){
        //     elevator.setElevatorPosition(elevator.getElevatorHeight(), -4);
        //     arm.setArmAngle(arm.getArmAngle(), -60);
        // }
        // else if(timer.get()> 1 && timer.get() < 2){
        //     arm.setArmAngle(arm.getArmAngle(), -60);
        // }
        // else{
        //     elevator.setElevatorPosition(elevator.getElevatorHeight(), -4);
        //     arm.setArmAngle(arm.getArmAngle(), -60);
        // }
        
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
        // elevator.setElevatorPosition(elevator.getElevatorHeight(), -4);
        // arm.setArmAngle(arm.getArmAngle(), -60);
    }
}
