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
public class AutonLowCube extends SequentialCommandGroup{
    public AutonLowCube(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist, Intake claw) {
        addCommands(
            new RollClawTimed(claw, 0.3, 0.5)
        );
            
    }
}
