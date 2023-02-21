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

      double rightY = gamepad.getRightY();
      
      double armInput = Util.deadband(0.1, rightY);

      arm.moveArm(armInput);
    }
  
    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
