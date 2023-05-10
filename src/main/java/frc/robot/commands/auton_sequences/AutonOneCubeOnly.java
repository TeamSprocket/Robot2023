
package frc.robot.commands.AutonSequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.macro.BalanceOnChargeStation;
import frc.robot.commands.macro.BalanceOnChargeStationGyro;
import frc.robot.commands.macro.timed.DeportArmTimed;
import frc.robot.commands.macro.timed.LimelightAlignTimed;
import frc.robot.commands.macro.timed.PIDTurnTimed;
import frc.robot.commands.macro.timed.PIDTurnTimedOnlyI;
import frc.robot.commands.macro.timed.RollClawTimed;
import frc.robot.commands.macro.timed.SetHighTimedCube;
import frc.robot.commands.macro.timed.SetHomeTimed;
import frc.robot.commands.macro.timed.SetLowCubeTimed;
import frc.robot.commands.macro.timed.SwerveDriveCmdTimed;
import frc.robot.commands.macro.timed.WaitTimed;
import frc.robot.commands.persistent.RollClaw;
import frc.robot.subsystems.SwerveDrive;

public class AutonOneCubeOnly extends SequentialCommandGroup {
  public AutonOneCubeOnly(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist, Claw claw) {
    // public AutonConeBalance(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist) {
    
    addCommands(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new DeportArmTimed(elevator, arm, wrist, 1),
          new RollClawTimed(claw, -0.2, 1)
        ), 

        // Put cube 
        new ParallelCommandGroup(
          new SetHighTimedCube(elevator, arm, wrist, 1),
          new RollClawTimed(claw, -0.25, 1),
          new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.45, new Rotation2d(0.0)), 1)
        ),

      // Score cube
        new ParallelCommandGroup(
          new RollClawTimed(claw, 0.25, 0.4),
          new SetHighTimedCube(elevator, arm, wrist, 0.4)
        ),
        new SetHighTimedCube(elevator, arm, wrist, 12.6)
      ));
      
  }
}
