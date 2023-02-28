
package frc.robot.commands.auton;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.SwerveDrive;
import frc.util.commands.MacroCommand;

public class SwerveDriveCmdTimed extends MacroCommand {

  private final SwerveDrive swerveDrive;
  private final double yTarget, xTarget, tTarget;
  private final Timer timer;
  private final double duration;

  /**
   * 
   * @param swerveDrive A SwerveDrive object
   * @param target Target Pose2d for the bot (given as x axis velocity from -1 to 1, y axis velocity from -1 to 1, and angular velocity in rad)
   * @param duration The number of seconds to traverse towards this target 
   * 
   */
  public SwerveDriveCmdTimed(SwerveDrive swerveDrive, Pose2d target, double duration) {
    this.swerveDrive = swerveDrive;
    this.xTarget = target.getX();
    this.yTarget = target.getY();
    this.tTarget = target.getRotation().getRadians();
    this.timer = new Timer();
    this.duration = duration;
  }

  // public atTarget() {
    // private SwerveModulePosition[] swerveModulePositions = {
      //     new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurnPosition())),
      //     new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurnPosition())),
      //     new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurnPosition())),
      //     new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurnPosition())),
      // };
  //   posEstimator = new SwerveDrivePoseEstimator(
  //     Constants.Drivetrain.driveKinematics, 
  //     new Rotation2d(Math.toRadians(swerveDrive.getHeading())), 
  //     null, 
  //     null);
      
  // }


  // Called when the command is initially scheduled
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("AUTON Y SPEED", yTarget);
    new SwerveDriveCmd(
      swerveDrive, 
      () -> yTarget, 
      () -> xTarget, 
      () -> tTarget
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() >= duration) {
      return true;
    } else {
        return false;
    }
  }
}
