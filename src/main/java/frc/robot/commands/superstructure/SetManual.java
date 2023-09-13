
package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperStructureState;

public class SetManual extends CommandBase {
  private Superstructure superstructure;

  public SetManual(Superstructure superstructure) {
    this.superstructure = superstructure;
  }

  @Override
  public void initialize() {
    superstructure.setState(SuperStructureState.MANUAL);
  }
  

  @Override
  public boolean isFinished() {
      return superstructure.reachedState();
  }

  @Override
  public void end(boolean interrupted) {}

}
