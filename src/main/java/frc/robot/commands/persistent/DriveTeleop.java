
package frc.robot.commands.persistent;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;

public class DriveTeleop extends CommandBase {

  private final SwerveDrive swerveDrive;
  private final Supplier<Double> xSPDFunct, ySPDFunct, tSPDFunct;
  private final Supplier<Boolean> fieldOrientedFunct;
  private final SlewRateLimiter xSlewLimit, ySlewLimit, tSlewLimit;

  /**
   * Used to send outputs to SwerveDrive
   * @param swerveDrive SwerveDrive object
   * @param xSPDFunct Vertical speed of the bot
   * @param ySPDFunct Horizontal speed of the bot
   * @param tSPDFunct Angular speed of the bot 
   */
  public DriveTeleop(SwerveDrive swerveDrive, Supplier<Double> xSPDFunct, Supplier<Double> ySPDFunct, Supplier<Double> tSPDFunct) {
      this.swerveDrive = swerveDrive;
      this.xSPDFunct = xSPDFunct;
      this.ySPDFunct = ySPDFunct;
      this.tSPDFunct = tSPDFunct;
      this.fieldOrientedFunct = () -> Constants.Drivetrain.IS_FIELD_ORIENTED;
      this.xSlewLimit = new SlewRateLimiter(Constants.Drivetrain.kMaxAccel);
      this.ySlewLimit = new SlewRateLimiter(Constants.Drivetrain.kMaxAccel);
      this.tSlewLimit = new SlewRateLimiter(Constants.Drivetrain.kMaxAccelAngular);
  }

  public static double deadband(double speed) {
    if (Math.abs(speed) < Constants.Drivetrain.kDriveTeleopDeadband) {
      return 0.0;
    }
    return speed;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = deadband(xSPDFunct.get());
    double ySpeed = deadband(ySPDFunct.get());
    double tSpeed = deadband(tSPDFunct.get());

    xSpeed = xSlewLimit.calculate(xSpeed);
    ySpeed = ySlewLimit.calculate(ySpeed);
    tSpeed = tSlewLimit.calculate(tSpeed);

    ChassisSpeeds chassisSpeeds;
    // Field Oriented
    if (Constants.Drivetrain.IS_FIELD_ORIENTED) {
      double headingRad = Math.toRadians(-swerveDrive.getHeading());
      
      if (Constants.Auton.FACING_DRIVERS) {
        headingRad += Math.PI;
      }

      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, tSpeed, new Rotation2d(headingRad));
    } else { // Robot oriented
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, tSpeed);
      SmartDashboard.putString("Chassis Speed", chassisSpeeds.toString());

    }

    // Calculate module states per module
    SwerveModuleState[] moduleStates = Constants.Measurements.driveKinematics.toSwerveModuleStates(chassisSpeeds);
    SmartDashboard.putString("Module States", moduleStates[2].toString());

    // Normalize speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.kMaxSpeed);

    // Apply to modules
    if (Constants.kIsTeleop) {
      swerveDrive.setModuleStates(moduleStates);
    }
    SmartDashboard.putString("Module States Desaturated", moduleStates[2].toString());

  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}


