package frc.robot.commands.macro;

import frc.robot.subsystems.Wrist;
import frc.util.commands.MacroCommand;

public class ResetWristEncoder extends MacroCommand {
    private final Wrist wrist;
  
    public ResetWristEncoder (Wrist wrist) {
      this.wrist = wrist;
  
      addRequirements(wrist);
    }

    public void initialize(){
        wrist.resetWristEncoder();
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