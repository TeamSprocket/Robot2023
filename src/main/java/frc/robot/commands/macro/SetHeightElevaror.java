
package frc.robot.commands.macro;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorReWrite;

public class SetHeightElevaror extends CommandBase {
  
  ElevatorReWrite elevator;
  private String direction;  

  public SetHeightElevaror(ElevatorReWrite elevator, String direction) {
    this.elevator = elevator;
    this.direction = direction;
  }

  @Override
  public void initialize() {
    if (direction.equals("up")) {
    elevator.setHeight(elevator.getHeight() +  Constants.Elevator.kHeightSetpointInterval);
  }
  else {
    elevator.setHeight(elevator.getHeight() -  Constants.Elevator.kHeightSetpointInterval);
  }

  }

  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
