
package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperStructureState;

public class SetHome extends CommandBase {
  private Superstructure superstructure;

  public SetHome(Superstructure superstructure) {
    this.superstructure = superstructure;
  }

  @Override
  public void initialize() {
    System.out.println("Command has started...");
    superstructure.setState(SuperStructureState.HOME);
  }
  

  @Override
  public boolean isFinished() {
      return superstructure.reachedState();
  }

  @Override
  public void end(boolean interrupted) {}

}
