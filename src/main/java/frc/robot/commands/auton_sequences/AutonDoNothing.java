
package frc.robot.commands.auton_sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.WaitTimed;


public class AutonDoNothing extends SequentialCommandGroup {
  public AutonDoNothing() {
    addCommands(
      new SequentialCommandGroup(

        new WaitTimed(15)
        
      )
    );
  }
}
