package frc.robot.commands.peresistent;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class MoveArmJoystick extends PersistentCommand {
    private final Arm arm;
    private final Elevator elevator;
    private final XboxController gamepad;
  
    public MoveArmJoystick (Arm arm, Elevator elevator, XboxController gamepad) {
        this.arm = arm;
        this.elevator = elevator;
        this.gamepad = gamepad;
  
        addRequirements(arm, elevator);
    }
  
    @Override
    public void execute() {

      double armMaxHeight;
      double armMinHeight;
      double armRange;

      double elevatorHeight = elevator.getElevtorHeight();
      if (elevatorHeight > 0){
        armMaxHeight = Constants.Arm.MAX_HEIGHT_METERS_ELEVATOR;
        armMinHeight = Constants.Arm.MIN_HEIGHT_METERS_ELEVATOR;
        armRange = Constants.Arm.HEIGHT_METERS_ELEVATOR;
      }
      else{
        armMaxHeight = Constants.Arm.MAX_HEIGHT_METERS_NO_ELEVATOR;
        armMinHeight = Constants.Arm.MIN_HEIGHT_METERS_NO_ELEVATOR;
        armRange = Constants.Arm.HEIGHT_METERS_NO_ELEVATOR;
      }

      double rightY = gamepad.getRightY();
      double deadbandedInput = Util.deadband(0.1, rightY);

      double targetHeight = (deadbandedInput) * armRange;


      //PID VERSION - adjust PID values
      arm.setArmPosition(targetHeight);

      //AFF version
      // arm.setArmPositionAFF(targetHeight);

      
        
    }
  
    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
