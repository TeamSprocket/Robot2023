package frc.robot.commands.peresistent;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class MoveWristManual extends PersistentCommand {
    private final Wrist wrist;
    private final XboxController gamepad;
  
    public MoveWristManual (Wrist wrist, XboxController gamepad) {
        this.wrist = wrist;
        this.gamepad = gamepad;
  
        addRequirements(wrist);
    }
  
    @Override
    public void execute() {

      double input = gamepad.getLeftTriggerAxis() - gamepad.getRightTriggerAxis(); 

      double deadbandedInput = Util.deadband(0.1, input);
      System.out.println(wrist.getWristAngle());
      
      wrist.moveWrist(deadbandedInput);
    }
  
    @Override
    public void end(boolean interrupted) {
        wrist.stop();
    }
}
