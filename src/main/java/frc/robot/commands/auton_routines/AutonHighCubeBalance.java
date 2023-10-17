
package frc.robot.commands.auton_routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.macro.BalanceOnChargeStation;
import frc.robot.commands.macro.auton.*;
import frc.robot.commands.superstructure.auton.RollClawTimed;
import frc.robot.commands.superstructure.auton.SetHighTimed;
import frc.robot.commands.superstructure.auton.SetHomeTimed;
import frc.robot.subsystems.*;

public class AutonHighCubeBalance extends SequentialCommandGroup {
  public AutonHighCubeBalance(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist, Claw claw) {
    addCommands( new SequentialCommandGroup(
      // Scoring high cube
      new ParallelCommandGroup(
        new SetHighTimed(elevator, arm, wrist, 2),
        new RollClawTimed(claw, -0.3, 1)
      ),
      new ParallelCommandGroup(
        new SetHighTimed(elevator, arm, wrist, 2.5),
        new SequentialCommandGroup(
          new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.5, new Rotation2d(0.0)), 1),
          new RollClawTimed(claw, 0.3, 0.5),
          new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.5, new Rotation2d(0.0)), 1)    
      )),
      new SetHomeTimed(elevator, arm, wrist, 2),
      new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.8, new Rotation2d(0.0)), 0.5),

      // Balance
      new BalanceOnChargeStation(swerveDrive, 0.8, 7.5)
      // new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.0, new Rotation2d(0.1)), 0.5)  

    ));
      
  }
}
