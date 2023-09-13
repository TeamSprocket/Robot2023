
package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperStructureState;

public class SetMidCone extends CommandBase {
  private Superstructure superstructure;

  public SetMidCone(Superstructure superstructure) {
    this.superstructure = superstructure;
  }

  @Override
  public void initialize() {
    superstructure.setState(SuperStructureState.MID_CONE);
  }
  

  @Override
  public boolean isFinished() {
      return superstructure.reachedState();
  }

  @Override
  public void end(boolean interrupted) {}

}
