
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
import frc.util.Util;

public class Drive extends CommandBase {

  private final SwerveDrive swerveDrive;
  private final Supplier<Double> xSPDFunct, ySPDFunct, tSPDFunct;
  private final SlewRateLimiter xSlewLimit, ySlewLimit, tSlewLimit;

  /**
   * Used to send outputs to SwerveDrive
   * @param swerveDrive SwerveDrive object
   * @param xSPDFunct Y axis speed of the bot
   * @param ySPDFunct X axis speed of the bot
   * @param tSPDFunct Angular speed of the bot 
   */
  public Drive(SwerveDrive swerveDrive, Supplier<Double> xSPDFunct, Supplier<Double> ySPDFunct, Supplier<Double> tSPDFunct) {
    this.swerveDrive = swerveDrive;
    this.xSPDFunct = xSPDFunct;
    this.ySPDFunct = ySPDFunct;
    this.tSPDFunct = tSPDFunct;
    this.xSlewLimit = new SlewRateLimiter(Constants.Drivetrain.kMaxAccelDrive);
    this.ySlewLimit = new SlewRateLimiter(Constants.Drivetrain.kMaxAccelDrive);
    this.tSlewLimit = new SlewRateLimiter(Constants.Drivetrain.kMaxAccelTurn);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (Constants.isEnabled) {
      // Speed with deadband
      double xSpeed = Util.deadband(0.075, xSPDFunct.get());
      double ySpeed = Util.deadband(0.075, ySPDFunct.get());
      double tSpeed = Util.deadband(0.075, tSPDFunct.get());

      // Slew limit (accel)
      xSpeed = xSlewLimit.calculate(xSpeed);
      ySpeed = ySlewLimit.calculate(ySpeed);
      tSpeed = tSlewLimit.calculate(tSpeed);

      // Module States
      Rotation2d headingRad = new Rotation2d(swerveDrive.getHeading()); // TODO: Check deleted negative in front of swerveDrive.getHeading() 
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, tSpeed, headingRad);
      SwerveModuleState[] moduleStates = Constants.Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);

      // Desaturate speeds
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.kMaxSpeedDrive);

      // Apply to modules
      swerveDrive.setModuleStates(moduleStates);
    }
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


