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
    private final double position;
  
    public MoveArmJoystick (Arm arm, Elevator elevator, double position) {
        this.arm = arm;
        this.elevator = elevator;
        this.position = position;
  
        addRequirements(arm, elevator);
    }
  
    @Override
    public void execute() {

      double elevatorHeight = elevator.getElevtorHeight();

      if (elevatorHeight < Constants.Arm.elevatorHeightRestriction){
        arm.setArmPosition(arm.getArmPosition());
      }
      else{
        if (position >=  Constants.Elevator.MIN_HEIGHT_METERS || position <= Constants.Elevator.MIN_HEIGHT_METERS){
          elevator.setElevatorHeight(elevator.getElevtorHeight());
        }
        else{
          elevator.setElevatorHeight(position);
        }
      }
      
        
    }
  
    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
