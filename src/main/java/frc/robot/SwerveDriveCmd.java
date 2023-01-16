
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;

public class SwerveDriveCmd extends CommandBase {

  private final SwerveDrive swerveDrive;
  private final Supplier<Double> xSPDFunct, ySPDFunct, tSPDFunct;
  private final Supplier<Boolean> fieldOrientedFunct;
  private final SlewRateLimiter xSlewLimit, ySlewLimit, tSlewLimit;

    public SwerveDriveCmd(SwerveDrive swerveDrive,
            Supplier<Double> xSPDFunct, Supplier<Double> ySPDFunct, Supplier<Double> tSPDFunct,
            Supplier<Boolean> fieldOrientedFunct) {

        this.swerveDrive = swerveDrive;
        this.xSPDFunct = xSPDFunct;
        this.ySPDFunct = ySPDFunct;
        this.tSPDFunct = tSPDFunct;
        this.fieldOrientedFunct = fieldOrientedFunct;
        this.xSlewLimit = new SlewRateLimiter(Constants.Drivetrain.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.ySlewLimit = new SlewRateLimiter(Constants.Drivetrain.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.tSlewLimit = new SlewRateLimiter(Constants.Drivetrain.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        
        addRequirements(swerveDrive);

    }

  public static double runDeadband(double speedDB) {
    if (Math.abs(speedDB) < 0.03) {
      return 0.0;
    }
    return speedDB;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    // Speed with deadband
    double xSpeed = runDeadband(xSPDFunct.get());
    double ySpeed = runDeadband(ySPDFunct.get());
    double tSpeed = runDeadband(tSPDFunct.get());

    // Slew limit (accel)
    xSpeed = xSlewLimit.calculate(xSpeed) * Constants.Drivetrain.kMaxSpeedMetersPerSecond;
    ySpeed = ySlewLimit.calculate(ySpeed) * Constants.Drivetrain.kMaxSpeedMetersPerSecond;
    tSpeed = tSlewLimit.calculate(tSpeed)
            * Constants.Drivetrain.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    
    ChassisSpeeds chassisSpeeds;
    // Field Oriented
    if (fieldOrientedFunct.get()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, tSpeed, swerveDrive.getRotation2d());
    }

    // Robot oriented
    else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, tSpeed);
    }

    // Calculate module states per module
    SwerveModuleState[] moduleStates = Constants.Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Apply to modules
    swerveDrive.setModuleStates(moduleStates);


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
