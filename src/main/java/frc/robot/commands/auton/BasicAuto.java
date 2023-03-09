// package frc.robot.commands.auton;

// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.armIntake.IntakeArmConeExtractCommand;
// import frc.robot.commands.armPositions.ArmConeHighCommand;
// import frc.robot.commands.swerve.SetSwerveIdleMode;
// import frc.robot.commands.swerve.SetSwerveOdometry;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.IntakeArm;

// public class BasicAuto extends SequentialCommandGroup {
//     public BasicAuto(DriveSubsystem swerveDrive, Arm arm, IntakeArm intakeArm){
        
//         addCommands(
//             new ArmConeHighCommand(arm).andThen(new IntakeArmConeExtractCommand(intakeArm)),


//             new SetSwerveIdleMode(swerveDrive, false)
//                 .andThen(() -> swerveDrive.drive(0, 0, 0, false)));
//     }
// } // <-- keep brace

