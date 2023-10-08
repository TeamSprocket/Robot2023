
package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.macro.SetHigh;
import frc.robot.commands.macro.timed.DeportArmTimed;
import frc.robot.commands.macro.timed.LimelightAlignTimed;
import frc.robot.commands.macro.timed.PIDTurnTimed;
import frc.robot.commands.macro.timed.RollClawTimed;
import frc.robot.commands.macro.timed.SetHighTimed;
// import frc.robot.commands.macro.timed.SetHighTimedCube;
import frc.robot.commands.macro.timed.SetHomeTimed;
import frc.robot.commands.macro.timed.SetLowCubeTimed;
import frc.robot.commands.macro.timed.SwerveDriveCmdTimed;
import frc.robot.commands.macro.timed.WaitTimed;
import frc.robot.commands.persistent.RollClaw;
import frc.robot.subsystems.SwerveDrive;

public class AutonHighOneCubeLeft extends SequentialCommandGroup {
  public AutonHighOneCubeLeft(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist, Claw claw) {
    addCommands(
      new SequentialCommandGroup(
        // Score
        new ParallelCommandGroup(
          new SetHighTimed(elevator, arm, wrist, 4.5),
          new SequentialCommandGroup(
            new RollClawTimed(claw, -1, 0.5),
            new WaitTimed(1.5),
            new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.5, new Rotation2d(0.0)), 1),
            new RollClawTimed(claw, 0.3, 0.5),
            new SwerveDriveCmdTimed(swerveDrive, new Pose2d(-0.3, 0.7, new Rotation2d(0.0)), 1.5)
          )
        ),

        // Set home
        new SetHomeTimed(elevator, arm, wrist, 2),
        new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.8, new Rotation2d(0.0)), 1.5),
        new PIDTurnTimed(swerveDrive, Math.PI, 1.5)
      )
    );
      
  }
}
