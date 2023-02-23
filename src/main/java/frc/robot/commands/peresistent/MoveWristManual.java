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

      double wristInput = Util.deadband(0.1, input);
      
    //   System.out.println("angle: " + wrist.getWristAngle());
    //   System.out.println("output: " + input);
      
      if (wristInput == 0){
        double output = wristInput;
        output += 0.000482 * wrist.getWristAngle() - 0.7029;
        
        // output += 8.7947 * Math.pow(10,-7) * wrist.getWristAngle() * wrist.getWristAngle() - 0.67095;
        
        wrist.moveWrist(output);
      }
      else{
        wrist.moveWrist(wristInput * 0.2);
      }
    }
  
    @Override
    public void end(boolean interrupted) {
        wrist.stop();
    }
}
