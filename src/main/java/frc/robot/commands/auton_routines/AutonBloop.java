
package frc.robot.commands.auton_routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.macro.auton.*;
import frc.robot.commands.superstructure.auton.*;
// import frc.robot.commands.macro.BalanceOnChargeStation;
import frc.robot.commands.macro.BalanceOnChargeStation;
import frc.robot.commands.persistent.RollClaw;
import frc.robot.subsystems.SwerveDrive;

public class AutonBloop extends SequentialCommandGroup {
  public AutonBloop(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist, Claw claw) {
    // public AutonConeBalance(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist) {
  
    addCommands(
      new SequentialCommandGroup(
        new RollClawTimed(claw, -1, 0.3),
        new WaitTimed(1.7),
        new RollClawTimed(claw, 1, 1)
      )
    );
  }
}
