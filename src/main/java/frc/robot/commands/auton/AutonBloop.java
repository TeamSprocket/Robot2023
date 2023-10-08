
package frc.robot.commands.auton;

// import org.apache.commons.collections4.bag.TreeBag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.macro.SetHigh;
import frc.robot.commands.macro.timed.DeportArmTimed;
import frc.robot.commands.macro.timed.LimelightAlignTimed;
import frc.robot.commands.macro.timed.PIDTurnTimed;
import frc.robot.commands.macro.timed.RollIntakeTimed;
import frc.robot.commands.macro.timed.SetHighTimed;
// import frc.robot.commands.macro.timed.SetHighTimedCube;
import frc.robot.commands.macro.timed.SetHomeTimed;
import frc.robot.commands.macro.timed.SetLowCubeTimed;
import frc.robot.commands.macro.timed.SwerveDriveCmdTimed;
import frc.robot.commands.macro.timed.WaitTimed;
import frc.robot.commands.persistent.RollIntake;
import frc.robot.subsystems.SwerveDrive;

public class AutonBloop extends SequentialCommandGroup {
  public AutonBloop(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    addCommands( new SequentialCommandGroup(
        new RollIntakeTimed(intake, -0.5, 0.5),
        new RollIntakeTimed(intake, 0.5, 1)
    ));
      
  }
}
