package frc.robot.commands.intake;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import frc.util.commands.MacroCommand;

public class MoveWristAngle extends MacroCommand {
    private final Wrist wrist;
    private final double angle;
  
    public MoveWristAngle (Wrist wrist, double angle) {
      this.wrist = wrist;
      this.angle = angle;
  
      addRequirements(wrist);
    }

    public void initialize(){
      
    }

    @Override
    public void execute() {
      wrist.setWristAngle(wrist.getWristAngle(), angle);
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
    }
}