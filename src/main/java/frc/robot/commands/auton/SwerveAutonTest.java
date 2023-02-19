// package frc.robot.commands.auton;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.SwerveDrive;


// public class SwerveAutonTest {
//     public SwerveAutonTest(SwerveDrive swerveDrive) {
//         addCommands(
//             new SequentialCommandGroup(
//                  new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0, 0.1, new Rotation2d(0.0)), 1),
//                  new WaitTimed(3),
//                  new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0, -0.1, new Rotation2d(0.0)), 1),
//             )
//         );
//     }
// }
