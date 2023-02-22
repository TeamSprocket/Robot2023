package frc.robot.commands.macro;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import frc.util.commands.MacroCommand;

public class MoveWristAngle extends MacroCommand {
    private final Wrist wrist;
    private final double operator;
  
    public MoveWristAngle (Wrist wrist, double operator) {
      this.wrist = wrist;
      this.operator = operator;
  
      addRequirements(wrist);
    }

    public void initialize(){
      
    }

    @Override
    public void execute() {
      wrist.setWristAngle(wrist.getWristAngle(), operator);
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
    }
}
