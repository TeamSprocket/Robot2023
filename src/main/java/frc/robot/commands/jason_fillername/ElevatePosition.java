package frc.robot.commands.jason_fillername;

import frc.robot.subsystems.Elevator;
import frc.util.commands.MacroCommand;

public class ElevatePosition extends MacroCommand {
    private final Elevator elevator;
    private final double height;
  
    public ElevatePosition (Elevator elevator, double height) {
      this.elevator = elevator;
      this.height = height;
  
      addRequirements(elevator);
    }

    public void initialize(){
      
    }

    @Override
    public void execute() {
      elevator.setElevatorPosition(elevator.getElevatorHeight(), height);
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
    }
}