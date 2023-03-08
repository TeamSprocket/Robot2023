
package frc.robot.commands.macro;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.SwerveDrive;
import frc.util.commands.MacroCommand;

public class SwerveDriveCmdPrecise extends MacroCommand {

  private final SwerveDrive swerveDrive;
  private final double yTarget, xTarget;
  private boolean first = true;
  private double startTime;
  private WPI_TalonFX talonFL;
  private WPI_TalonFX talonFR;
  private WPI_TalonFX talonBL;
  private WPI_TalonFX talonBR;
  private Timer timer;

  SlewRateLimiter xSlewLimit, ySlewLimit, tSlewLimit;

  // private RunMotorPersistent runMotorPersistentFL, runMotorPersistentFR, runMotorPersistentBL, runMotorPersistentBR;

  double xSpeed;
  double ySpeed;

  /**
   * 
   * @param swerveDrive A SwerveDrive object
   * @param target Target Pose2d for the bot (given as x axis velocity from -1 to 1, y axis velocity from -1 to 1, and angular velocity in rad)
   * @param duration The number of seconds to traverse towards this target 
   * 
   */
  public SwerveDriveCmdPrecise(SwerveDrive swerveDrive, double x, double y) {
    this.swerveDrive = swerveDrive;
    this.xTarget = y;
    this.yTarget = x;
    
    this.xSpeed = xTarget / 5;
    this.ySpeed = yTarget / 5;


  }

    
      
  // Called when the command is initially scheduled
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.start();

    ChassisSpeeds chassisSpeeds;
    // // Field Oriented
    if (Constants.Drivetrain.IS_FIELD_ORIENTED) {
      double headingRad = Math.toRadians(swerveDrive.getHeading());
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, 0.0, new Rotation2d(headingRad));
    } else { // Robot oriented
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0.0);
    }

    // Calculate module states per module
    SwerveModuleState[] moduleStates = Constants.Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    // Apply to modules
    swerveDrive.setModuleStates(moduleStates);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopModules();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() >= 0.2) {
      return true;
    }
    return false;
  }

}
