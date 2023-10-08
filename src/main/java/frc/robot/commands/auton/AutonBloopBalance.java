
package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.macro.BalanceOnChargeStation;
import frc.robot.commands.macro.timed.*;
import frc.robot.subsystems.*;

public class AutonBloopBalance extends SequentialCommandGroup {
  public AutonBloopBalance(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    addCommands( new SequentialCommandGroup(
      // new RollIntakeTimed(intake, -1, 0.25),
      // new RollIntakeTimed(intake, 0.3, 0.5),

    // Balance
    new BalanceOnChargeStation(swerveDrive, 0.8, 1300)
    // new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.0, new Rotation2d(0.1)), 0.5)  

    ));
      
  }
}
