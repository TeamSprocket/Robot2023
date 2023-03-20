
package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.macro.BalanceOnChargeStation;
import frc.robot.commands.macro.timed.DeportArmTimed;
import frc.robot.commands.macro.timed.LimelightAlignTimed;
import frc.robot.commands.macro.timed.RollClawTimed;
import frc.robot.commands.macro.timed.SetHighTimed;
import frc.robot.commands.macro.timed.SwerveDriveCmdTimed;
import frc.robot.commands.macro.timed.WaitTimed;
import frc.robot.commands.persistent.RollClaw;
import frc.robot.subsystems.SwerveDrive;

public class AutonBalance extends SequentialCommandGroup {
  public AutonBalance(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist, Claw claw) {
    // public AutonConeBalance(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist) {
    
    addCommands(
      new SequentialCommandGroup(
        // new ParallelCommandGroup(
        //   new SequentialCommandGroup(
        //     new DeportArmTimed(elevator, arm, wrist, 1),
        //     new SetHighTimed(elevator, arm, wrist, 2),
        //     new LimelightAlignTimed(swerveDrive, -0.4, 2)
        //   ),
        //   new RollClawTimed(claw, 0.25, 5)
        // ),
        // new RollClawTimed(claw, -0.5, 1),
        // new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.5, new Rotation2d(0.0)), 2),
        // new BalanceOnChargeStation(swerveDrive, 0.5, true)
        new RollClawTimed(claw, 0.25, 1),
        new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.7, new Rotation2d(0.0)), 4.25),
        new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.5, new Rotation2d(0.0)), 0.25),
        new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.3, new Rotation2d(0.0)), 0.25),
        new WaitTimed(0.5),
        new BalanceOnChargeStation(swerveDrive, -0.085, true),
        new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.0, new Rotation2d(0.1)), 0.5)
        
      )
    );
  }
}
