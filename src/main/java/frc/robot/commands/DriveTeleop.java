
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.util.Util;

public class DriveTeleop extends CommandBase {
  private final SwerveDrive swerveDrive;
  private final Supplier<Double> xSupplier, ySupplier, tSupplier;
  private final SlewRateLimiter xSlewLimit, ySlewLimit, tSlewLimit;

  public DriveTeleop(SwerveDrive swerveDrive, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> tSupplier) {
    this.swerveDrive = swerveDrive;
    this.xSupplier = ySupplier; // PURPOSEFULLY SWITCHED
    this.ySupplier = xSupplier; // PURPOSEFULLY SWITCHED
    this.tSupplier = tSupplier;
    this.xSlewLimit = new SlewRateLimiter(Constants.Drivetrain.kMaxAccel);
    this.ySlewLimit = new SlewRateLimiter(Constants.Drivetrain.kMaxAccel);
    this.tSlewLimit = new SlewRateLimiter(Constants.Drivetrain.kMaxTurnAccel);
      
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {}
  
  
  @Override
  public void execute() {
    double xSpeed = Util.deadband(xSupplier.get(), 0.2);
    // double xSpeed = 0;
    double ySpeed = Util.deadband(ySupplier.get(), 0.2);
    // double tSpeed = 0;
    double tSpeed = Util.deadband(tSupplier.get(), 0.2);
    
    xSpeed = xSlewLimit.calculate(xSpeed) * Constants.Drivetrain.kMaxSpeed;
    ySpeed = ySlewLimit.calculate(ySpeed) * Constants.Drivetrain.kMaxSpeed;
    tSpeed = tSlewLimit.calculate(tSpeed) * Constants.Drivetrain.kMaxTurnSpeed;

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, tSpeed, new Rotation2d(swerveDrive.getHeading()));
    SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.kMaxSpeed);
    swerveDrive.setModuleStates(moduleStates);
  
  }
  

}
