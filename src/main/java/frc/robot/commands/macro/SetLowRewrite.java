
package frc.robot.commands.macro;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorReWrite;

public class SetLowRewrite extends CommandBase {
  
  ElevatorReWrite elevator;

  public SetLowRewrite(ElevatorReWrite elevator) {
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
    elevator.setHeight(Constants.Elevator.kHeightSetpointLow);
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
