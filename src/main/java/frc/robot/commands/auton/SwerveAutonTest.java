// package frc.robot.commands.auton;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.macro.SwerveDriveCmdTimed;
// import frc.robot.subsystems.SwerveDrive;
// import frc.util.commands.AutonRoutine;


// // public class SwerveAutonTest extends AutonRoutine {
// public class SwerveAutonTest extends CommandBase{
//     SwerveDrive swerveDrive;
//     public SwerveAutonTest(SwerveDrive swerveDrive) {
//         this.swerveDrive = swerveDrive;
//         Timer timer = new Timer();
//         while (timer.get() < 1) {
//             new SwerveDriveCmd(() -> swerveDrive, () -> -0.0, () -> 0.0, () -> new Rotation2d(0.0));
//         }

//             // new SequentialCommandGroup(
//                 //  new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0, 0.1, new Rotation2d(0.0)), 0.5),
//                 //  new WaitTimed(3),
//                 //  new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0, -0.1, new Rotation2d(0.0)), 0.5)
//         // );
//     }
// }
