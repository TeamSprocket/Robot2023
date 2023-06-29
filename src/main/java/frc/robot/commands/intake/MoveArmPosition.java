package frc.robot.commands.intake;

import frc.robot.subsystems.Arm;
import frc.util.commands.MacroCommand;

public class MoveArmPosition extends MacroCommand {
    private final Arm arm;
    private final double angle;
  
    public MoveArmPosition (Arm arm, double angle) {
      this.arm = arm;
      this.angle = angle;
  
      addRequirements(arm);
    }

    public void initialize(){
      
    }

    @Override
    public void execute() {
      arm.setArmAngle(arm.getArmAngle(), angle);
    }

    public boolean isFinished(){
      return false;
      
    }

    @Override
    public void end(boolean interrupted){
    }
}