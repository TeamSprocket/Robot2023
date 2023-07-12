
package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonDoNothing extends SequentialCommandGroup {
  public AutonDoNothing() {
    addCommands(
      new SequentialCommandGroup()
    );
  }
}
