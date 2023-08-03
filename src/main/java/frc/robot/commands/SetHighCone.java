
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Wrist.WristStates;


public class SetHighCone extends CommandBase {
  Elevator elevator;
  Arm arm;
  Wrist wrist;

  public SetHighCone(Elevator elevator, Arm arm, Wrist wrist) {
    this.elevator = elevator;
    this.arm = arm;
    this.wrist = wrist;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Wait for elevator to be clear
    if (!elevator.reachedClearHeight()) {
      elevator.setState(ElevatorStates.CLEAR);
    } else {
      elevator.setState(ElevatorStates.HIGH_CONE);
      arm.setState(ArmStates.HIGH_CONE);
      wrist.setState(WristStates.HIGH_CONE);
    }
  }
  

  @Override
  public boolean isFinished() {
      return elevator.reachedStatePos() && arm.reachedStatePos() && wrist.reachedStatePos();
  }

  @Override
  public void end(boolean interrupted) {}

}
