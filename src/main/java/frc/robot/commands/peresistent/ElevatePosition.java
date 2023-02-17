package frc.robot.commands.peresistent;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class ElevatePosition extends PersistentCommand {
    private final Elevator elevator;
    private final double position;
  
    public ElevatePosition (Elevator elevator, double position) {
      this.elevator = elevator;
      this.position = position;
  
      addRequirements(elevator);
    }
  
    @Override
    public void execute() {
      
      if (position >=  Constants.Elevator.MIN_HEIGHT_METERS || position <= Constants.Elevator.MIN_HEIGHT_METERS){
        elevator.setElevatorHeight(elevator.getElevtorHeightInMeters());
      }
      else{
        elevator.setElevatorHeight(position);
      }

      // if (encoder >=  Constants.Elevator.MAX_ENCODER_VALUE){
      //   elevator.setElevatorHeightEncoder(Constants.Elevator.MAX_ENCODER_VALUE);
      // }
      // else if (encoder <=  Constants.Elevator.MIN_ENCODER_VALUE){
      //   elevator.setElevatorHeightEncoder(Constants.Elevator.MIN_ENCODER_VALUE);
      // }        
      // else{
      //   elevator.setElevatorHeightEncoder(encoder);
      // }
        
    }
  
    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
