package frc.robot.commands.peresistent;

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

      double rightY = gamepad.getLeftY();

      // double rightY = gamepad.getRightY();
      double armInput = Util.deadband(0.1, rightY);
      //   System.out.println(rightY);
      // System.out.println(arm.getArmAngle());
      
      // if (armInput == 0) {
        // double output = armInput;
        // output += 0.00224556 * arm.getArmAngle() + 0.103286;
        // if (arm.getArmAngle() > -60 && arm.getArmAngle() < 0) {
        //   output -= 0.03;
        //   if (arm.getArmAngle() > -30 && arm.getArmAngle() < 0) {
        //     output -= 0.01;
        //   }
        // }
      //   arm.moveArm(output);
      // }
      // else {
      //   arm.moveArm(armInput);
      // }
      double newSetpoint = (5 * armInput) + arm.getArmAngle();
      // System.out.println("ARM ANGLE: " + arm.getArmAngle());
      if (armInput == 0){
        // if (arm.getArmAngle() < 0 && arm.getArmAngle() > -1) {
        //   arm.setArmAngle(arm.getArmAngle(), newSetpoint+0.5);
        // }
        if (arm.getArmAngle() < -1 && arm.getArmAngle() > -4.5) {
          double output = armInput;
          output += 0.00224556 * arm.getArmAngle() + 0.103286;
          //if (arm.getArmAngle() > -60 && arm.getArmAngle() < 0) {
            output -= 0.03;
          //if (arm.getArmAngle() > -30 && arm.getArmAngle() < 0) {
            output -= 0.01;
          //}
          arm.moveArm(output);
        }
        
        
        else if (arm.getArmAngle() < -4.5 && arm.getArmAngle() > -5.3) {
          arm.setArmAngle(arm.getArmAngle(), newSetpoint+0.5);
        }
        else if (arm.getArmAngle() < -5.5 && arm.getArmAngle() > -6.4){
          arm.setArmAngle(arm.getArmAngle(), newSetpoint-1.5);
        }
        else if (arm.getArmAngle() < -6.4) {
          arm.setArmAngle(arm.getArmAngle(), newSetpoint-1.8);
        }
        else {
          arm.setArmAngle(arm.getArmAngle(), newSetpoint-1);
        }
        //arm.setArmAngle(arm.getArmAngle(), newSetpoint-1);
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
