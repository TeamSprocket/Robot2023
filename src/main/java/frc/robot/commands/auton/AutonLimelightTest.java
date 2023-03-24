
package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.macro.timed.LimelightAlignTimed;
import frc.robot.commands.macro.timed.WaitTimed;
import frc.robot.subsystems.SwerveDrive;

public class AutonLimelightTest extends SequentialCommandGroup {
  public AutonLimelightTest(SwerveDrive swerveDrive) {
    addCommands(
      new SequentialCommandGroup(
        new LimelightAlignTimed(swerveDrive, 1, 15)
      )
    );
  }
}
