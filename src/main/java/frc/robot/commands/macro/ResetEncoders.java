package frc.robot.commands.macro;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.util.commands.MacroCommand;

public class ResetEncoders extends MacroCommand {
    private final Wrist wrist;
    private final Arm arm;
    private final Elevator elevator;
  
    public ResetEncoders (Elevator elevator, Arm arm, Wrist wrist) {
      this.wrist = wrist;
      this.arm = arm;
      this.elevator = elevator;
  
      addRequirements(wrist);
    }

    public void initialize(){
        wrist.resetWristEncoder();
        arm.resetArmEncoder();
        elevator.resetElevatorEncoder();
    }

    @Override
    public void execute() {
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
    }
}