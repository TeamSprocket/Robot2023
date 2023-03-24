
package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.macro.timed.WaitTimed;

public class AutonDoNothing extends SequentialCommandGroup {
  public AutonDoNothing() {
    addCommands(
      new SequentialCommandGroup(
        new WaitTimed(15)
      )
    );
  }
}
