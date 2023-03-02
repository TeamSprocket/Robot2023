
package frc.robot.commands.macro;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.SwerveDrive;
import frc.util.commands.MacroCommand;

public class SwerveDriveCmdTimed extends MacroCommand {

  private final SwerveDrive swerveDrive;
  private final double yTarget, xTarget, tTarget;
  private final Timer timer;
  private final double duration;
  private boolean first = true;
  private double startTime;
  private WPI_TalonFX talon;

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
    talon = new WPI_TalonFX(RobotMap.Drivetrain.FRONT_LEFT_TALON_D);
    
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
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (first) {
    timer.start();
    
      // first = false;
    // }
    
    new SwerveDriveCmd(
      swerveDrive, 
      () -> yTarget, 
      () -> xTarget, 
      () -> tTarget
    );
    // new SwerveDriveCmd(
    //   swerveDrive, 
    //   () -> 0.5, 
    //   () -> 0.0, 
    //   () -> 0.0
    // );

    // System.out.println("SWERVE IS RUNNING\nSWERVE IS RUNNING\nSWERVE IS RUNNING\nSWERVE IS RUNNING\nSWERVE IS RUNNING\n");
    // for (int i = 0; i < 10; i++) {
    //   System.out.println("SWERVE TIME: " + (Timer.getFPGATimestamp() - startTime));
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopModules();
      // for (int i = 0; i < 100; i++) {
      //     System.out.println("END SWERVE END SWERVE END SWERVE");
      // }
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
