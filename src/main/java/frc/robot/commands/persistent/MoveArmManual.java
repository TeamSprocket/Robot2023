package frc.robot.commands.persistent;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.util.Util;
import frc.util.wpilib_defaults.commands.PersistentCommand;

public class MoveArmManual extends PersistentCommand {
    private final Arm arm;
    private final XboxController gamepad;
  
    public MoveArmManual (Arm arm, XboxController gamepad) {
        this.arm = arm;
        this.gamepad = gamepad;
  
        addRequirements(arm);
    }
  
    @Override
    public void execute() {

      // += decreases output (goes down)
      // -= increases output (goes up)

      double input = gamepad.getRightY();

      double armInput = Util.deadband(0.1, input);

      double newSetpoint = (5 * armInput) + arm.getArmAngle();
      //System.out.println("ARM ANGLE: " + arm.getArmAngle());
      if (armInput == 0){
        if (arm.getArmAngle() < -1 && arm.getArmAngle() > -80) {
          double output = armInput;
          output += 0.00224556 * arm.getArmAngle() + 0.15;
          
          if (arm.getArmAngle() > -50) {
            output -= 0.06;
            if (arm.getArmAngle() > -25) {
              output -= 0.050;
            } 
            if (arm.getArmAngle() > -10) {
              output -= 0.01;
            } 

          }
          if (arm.getArmAngle() < -50 && arm.getArmAngle() > -65) {
            output -= 0.025;
            
          }
          if (arm.getArmAngle() < -65) {
            output -= 0.015;
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
