
package frc.robot.commands.AutonSequences;

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
