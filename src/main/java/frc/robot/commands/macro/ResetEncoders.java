package frc.robot.commands.macro;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.util.commands.MacroCommand;

public class ResetEncoders extends MacroCommand {
    private final Wrist wrist;
    private final Arm arm;
    private final Elevator elevator;
    private final Timer timer;
  
    public ResetEncoders (Elevator elevator, Arm arm, Wrist wrist) {
      this.wrist = wrist;
      this.arm = arm;
      this.elevator = elevator;

      timer = new Timer();
  
      addRequirements(wrist);
    }

    public void initialize(){
        timer.reset();
    }

    @Override
    public void execute() {
      timer.start(); //TODO
      if (timer.get() > 0 && timer.get() < 0.25){
        arm.moveArm(-0.2);
        elevator.moveElevator(-0.25);
      } else {
          arm.moveArm(-0.2);
          elevator.moveElevator(0.25);
      }
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
      wrist.resetWristEncoder();
      arm.resetArmEncoder();
      elevator.resetElevatorEncoder();
    }
}
