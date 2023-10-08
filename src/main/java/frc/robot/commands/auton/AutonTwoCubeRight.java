
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
import frc.robot.commands.macro.timed.RollIntakeTimed;
import frc.robot.commands.macro.timed.SetHighTimed;
// import frc.robot.commands.macro.timed.SetHighTimedCube;
import frc.robot.commands.macro.timed.SetHomeTimed;
import frc.robot.commands.macro.timed.SetLowCubeTimed;
import frc.robot.commands.macro.timed.SwerveDriveCmdTimed;
import frc.robot.commands.macro.timed.WaitTimed;
import frc.robot.commands.persistent.RollIntake;
import frc.robot.subsystems.SwerveDrive;

public class AutonTwoCubeRight extends SequentialCommandGroup {
  public AutonTwoCubeRight(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    addCommands(
      new SequentialCommandGroup(
        // Score
        new ParallelCommandGroup(
          new SetHighTimed(elevator, arm, wrist, 4),
          new SequentialCommandGroup(
            new RollIntakeTimed(intake, -1, 0.5),
            new WaitTimed(1),
            new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.5, new Rotation2d(0.0)), 1.5),
            new RollIntakeTimed(intake, 0.3, 0.5),
            new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.35, 0.7, new Rotation2d(0.0)), 0.5)
          )
        ),

        // Set home
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.35, 0.7, new Rotation2d(0.0)), 1),
            new WaitTimed(1)
          ),
          new SetHomeTimed(elevator, arm, wrist, 2)
        ),

        new SwerveDriveCmdTimed(swerveDrive, new Pose2d(-0.3, 0.75, new Rotation2d(0.0)), 0.5),
        new PIDTurnTimed(swerveDrive, ((13 * Math.PI) / 12.0), 1.5),

        new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.5, new Rotation2d(0.0)), 0.2),
        new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.9, new Rotation2d(0.0)), 0.8),

        new ParallelCommandGroup(
          new SwerveDriveCmdTimed(swerveDrive, new Pose2d(-0.16, 0.6, new Rotation2d(0.0)), 3.5),
          new SequentialCommandGroup(
            new SetHighTimed(elevator, arm, wrist, 0.5),
            new SetLowCubeTimed(elevator, arm, wrist, 3)
          ),
          new RollIntakeTimed(intake, -1.0, 3.5)
        ),

        new SetHomeTimed(elevator, arm, wrist, 2)


      )
    );
      
  }
}
