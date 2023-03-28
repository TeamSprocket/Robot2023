
package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.macro.BalanceOnChargeStation;
import frc.robot.commands.macro.BalanceOnChargeStationGyro;
import frc.robot.commands.macro.BalanceOnChargeStationGyroReverse;
import frc.robot.commands.macro.timed.DeportArmTimed;
import frc.robot.commands.macro.timed.LimelightAlignTimed;
import frc.robot.commands.macro.timed.PIDTurnTimed;
import frc.robot.commands.macro.timed.PIDTurnTimedOnlyI;
import frc.robot.commands.macro.timed.RollClawTimed;
import frc.robot.commands.macro.timed.SetHighTimed;
import frc.robot.commands.macro.timed.SetHighTimedCube;
import frc.robot.commands.macro.timed.SetHomeTimed;
import frc.robot.commands.macro.timed.SetLowCubeTimed;
import frc.robot.commands.macro.timed.SwerveDriveCmdTimed;
import frc.robot.commands.macro.timed.WaitTimed;
import frc.robot.commands.persistent.RollClaw;
import frc.robot.subsystems.SwerveDrive;

public class AutonOneHighCubeBalance extends SequentialCommandGroup {
  public AutonOneHighCubeBalance(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist, Claw claw) {
    // public AutonConeBalance(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist) {
    
    addCommands(
      new SequentialCommandGroup(
        // Approach grid
        new ParallelCommandGroup(
            new SequentialCommandGroup(
              new DeportArmTimed(elevator, arm, wrist, 1),
              // new SetHighTimedCube(elevator, arm, wrist, 1.5),
              new SetHighTimed(elevator, arm, wrist, 1.5),
              new ParallelCommandGroup(
                new SetHighTimed(elevator, arm, wrist, 0.5),
                new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.5, new Rotation2d(0.0)), 0.5)
              )
            ),
            new RollClawTimed(claw, -0.2, 3)
        ),
        
        // High bloop
        new ParallelCommandGroup(
          new RollClawTimed(claw, 0.25, 0.25),
          new SetHighTimed(elevator, arm, wrist, 0.25)
        ),
        
        // Home
        new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.1, new Rotation2d(0.0)), 0.1),
        new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.25, new Rotation2d(0.0)), 0.1),
        new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.4, new Rotation2d(0.0)), 0.1),
        new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.55, new Rotation2d(0.0)), 0.4),
        
        new SetHomeTimed(elevator, arm, wrist, 1.5),

        // new PIDTurnTimed(swerveDrive, Math.PI, 1),

        // new BalanceOnChargeStationGyro(swerveDrive, 0.08, true, 7),
        new BalanceOnChargeStationGyroReverse(swerveDrive, 0.08, true, 8),

        new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.0, new Rotation2d(0.1)), 0.5)        

      ));
      
  }
}
