package frc.robot.commands.persistent;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class MoveArmJoystick extends PersistentCommand {
    private final Arm arm;
    private final XboxController gamepad;
  
    public MoveArmJoystick (Arm arm, XboxController gamepad) {
        this.arm = arm;
        this.gamepad = gamepad;
  
        addRequirements(arm);
    }
  
    @Override
    public void execute() {

      double input = gamepad.getRightY();

      double armInput = Util.deadband(0.1, input);

      double newSetpoint = (5 * armInput) + arm.getArmAngle();
      // System.out.println("ARM ANGLE: " + arm.getArmAngle());
      if (armInput == 0){
        if (arm.getArmAngle() < -1 && arm.getArmAngle() > -80) {
          double output = armInput;
          output += 0.00224556 * arm.getArmAngle() + 0.083286;
          if (arm.getArmAngle() > -28) {
            output -= 0.025;
            if (arm.getArmAngle() > -15) {
              output -= 0.020;
            } 
            if (arm.getArmAngle() > -1) {
              output += 0.015;
            } 

          }
          if (arm.getArmAngle() < -65) {
            output += 0.025;
          }

          arm.moveArm(output);
        }
        
      
      }
      else{
        
        arm.setArmAngleManual(arm.getArmAngle(), newSetpoint);
      }
    }

    
  
    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
