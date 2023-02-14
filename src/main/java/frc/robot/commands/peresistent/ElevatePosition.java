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
        elevator.setElevatorHeight(position);

      // if (deadbandedInput > 0 && elevator.getElevtorHeightInMeters() > Constants.Elevator.MAX_HEIGHT_METERS){
      //   deadbandedInput = 0;
      // }
      // else if (deadbandedInput < 0 && elevator.getElevtorHeightInMeters() < Constants.Elevator.MIN_HEIGHT_METERS){
      //   deadbandedInput = 0;
      // }
      // else{
      //   elevator.setOutputManual(deadbandedInput);
      // }
    }
  
    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
