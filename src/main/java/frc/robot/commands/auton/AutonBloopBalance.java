
package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
// import frc.robot.commands.macro.BalanceOnChargeStation;
import frc.robot.commands.macro.BalanceOnChargeStation;
import frc.robot.commands.macro.timed.DeportArmTimed;
import frc.robot.commands.macro.timed.LimelightAlignTimed;
import frc.robot.commands.macro.timed.PIDTurnTimed;
import frc.robot.commands.macro.timed.RollIntakeTimed;
import frc.robot.commands.macro.timed.SetHighTimedCube;
import frc.robot.commands.macro.timed.SwerveDriveCmdTimed;
import frc.robot.commands.macro.timed.WaitTimed;
import frc.robot.commands.macro.timed.ZeroHeadingTimed;
import frc.robot.commands.persistent.RollIntake;
import frc.robot.subsystems.SwerveDrive;

public class AutonBloopBalance extends SequentialCommandGroup {
  public AutonBloopBalance(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    // public AutonConeBalance(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist) {
  
    addCommands(
      new SequentialCommandGroup(
        new RollIntakeTimed(intake, -1, 0.25),
        new RollIntakeTimed(intake, 1, 0.25),
        new PIDTurnTimed(swerveDrive, Math.PI, 1.5),
        new BalanceOnChargeStation(swerveDrive, 0.08, true, 12),
        new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.0, new Rotation2d(0.1)), 0.5)
      )
    );
  }
}
