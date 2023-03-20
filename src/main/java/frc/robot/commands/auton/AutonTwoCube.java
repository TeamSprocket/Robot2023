
package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.macro.BalanceOnChargeStation;
import frc.robot.commands.macro.timed.DeportArmTimed;
import frc.robot.commands.macro.timed.LimelightAlignTimed;
import frc.robot.commands.macro.timed.PIDTurnTimed;
import frc.robot.commands.macro.timed.RollClawTimed;
import frc.robot.commands.macro.timed.SetHighTimed;
import frc.robot.commands.macro.timed.SetHomeTimed;
import frc.robot.commands.macro.timed.SetLowCubeTimed;
import frc.robot.commands.macro.timed.SwerveDriveCmdTimed;
import frc.robot.commands.macro.timed.WaitTimed;
import frc.robot.commands.persistent.RollClaw;
import frc.robot.subsystems.SwerveDrive;

public class AutonTwoCube extends SequentialCommandGroup {
  public AutonTwoCube(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist, Claw claw) {
    // public AutonConeBalance(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist) {
    
    addCommands(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
              new DeportArmTimed(elevator, arm, wrist, 1),
              new SetHighTimed(elevator, arm, wrist, 2),
              new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.5, new Rotation2d(0.0)), 0.5)
            ),
            new RollClawTimed(claw, -0.2, 3)
        ),
          new ParallelCommandGroup(
            new SetHighTimed(elevator, arm, wrist, 3),
            new SequentialCommandGroup(
              new RollClawTimed(claw, 0.5, 1),
              new SwerveDriveCmdTimed(swerveDrive, new Pose2d(-0.05, 0.8, new Rotation2d(0.0)), 1),
              new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.1, 0.8, new Rotation2d(0.0)), 1)
            )
        ),
        new SetHomeTimed(elevator, arm, wrist, 2),
        new PIDTurnTimed(swerveDrive, Math.PI, 1),
        new DeportArmTimed(elevator, arm, wrist, 1),
        new ParallelCommandGroup(
          new SetLowCubeTimed(elevator, arm, wrist, 1),
          new RollClawTimed(claw, -1, 1),
          new SwerveDriveCmdTimed(swerveDrive, new Pose2d(-0.1, 0.33, new Rotation2d(0.0)), 1)
        ),
        new SetHomeTimed(elevator, arm, wrist, 2)
        // new PIDTurnTimed(swerveDrive, 0 + (Math.PI / 6), 2)

      ));
      
  }
}
