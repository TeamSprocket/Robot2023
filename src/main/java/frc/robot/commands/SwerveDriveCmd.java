
package frc.robot.commands;

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

public class SwerveDriveCmd extends CommandBase {

  private final SwerveDrive swerveDrive;
  private final Supplier<Double> xSPDFunct, ySPDFunct, tSPDFunct;
  private final Supplier<Boolean> fieldOrientedFunct;
  private final SlewRateLimiter xSlewLimit, ySlewLimit, tSlewLimit;

    public SwerveDriveCmd(SwerveDrive swerveDrive,
            Supplier<Double> xSPDFunct, Supplier<Double> ySPDFunct, Supplier<Double> tSPDFunct) {

        this.swerveDrive = swerveDrive;
        this.xSPDFunct = xSPDFunct;
        this.ySPDFunct = ySPDFunct;
        this.tSPDFunct = tSPDFunct;
        this.fieldOrientedFunct = () -> Constants.Drivetrain.IS_FIELD_ORIENTED;
        this.xSlewLimit = new SlewRateLimiter(Constants.Drivetrain.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.ySlewLimit = new SlewRateLimiter(Constants.Drivetrain.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.tSlewLimit = new SlewRateLimiter(Constants.Drivetrain.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        
        addRequirements(swerveDrive);

    }

  public static double runDeadband(double speedDB) {
    if (Math.abs(speedDB) < 0.075) {
      return 0.0;
    }
    return speedDB;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    // Update Position Estimation
    swerveDrive.updatePos();


    // Speed with deadband
    double xSpeed = runDeadband(xSPDFunct.get());
    double ySpeed = runDeadband(ySPDFunct.get());
    double tSpeed = runDeadband(tSPDFunct.get());

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("tSpeed", tSpeed);

    // Slew limit (accel)
    xSpeed = xSlewLimit.calculate(xSpeed) * Constants.Drivetrain.kMaxSpeedMetersPerSecond;
    ySpeed = ySlewLimit.calculate(ySpeed) * Constants.Drivetrain.kMaxSpeedMetersPerSecond;
    tSpeed = tSlewLimit.calculate(tSpeed) * Constants.Drivetrain.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    
    SmartDashboard.putNumber("SLEW xSpeed", xSpeed);
    SmartDashboard.putNumber("SLEW ySpeed", ySpeed);
    SmartDashboard.putNumber("SLEW tSpeed", tSpeed);

    ChassisSpeeds chassisSpeeds;
    // Field Oriented
    if (Constants.Drivetrain.IS_FIELD_ORIENTED) {
      double headingRad = Math.toRadians(swerveDrive.getHeading());
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, tSpeed, new Rotation2d(headingRad));
    } else { // Robot oriented
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, tSpeed);
      SmartDashboard.putString("Chassis Speed", chassisSpeeds.toString());

    }

    // Calculate module states per module
    SwerveModuleState[] moduleStates = Constants.Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);
    SmartDashboard.putString("Module States", moduleStates[2].toString());

    // Normalize speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.kMaxSpeedMetersPerSecond);

    // Apply to modules
    swerveDrive.setModuleStates(moduleStates);
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



