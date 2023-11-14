
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

public class SwerveDriveCmd extends CommandBase {

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
    public SwerveDriveCmd(SwerveDrive swerveDrive,
            Supplier<Double> xSPDFunct, Supplier<Double> ySPDFunct, Supplier<Double> tSPDFunct) {

          // System.out.println("SWERVE DRIVE CMD STARTED!!!!!!!!!\nSWERVE DRIVE CMD STARTED!!!!!!!!!\nSWERVE DRIVE CMD STARTED!!!!!!!!!\nSWERVE DRIVE CMD STARTED!!!!!!!!!\nSWERVE DRIVE CMD STARTED!!!!!!!!!\nSWERVE DRIVE CMD STARTED!!!!!!!!!\n");
        this.swerveDrive = swerveDrive;
        this.xSPDFunct = xSPDFunct;
        this.ySPDFunct = ySPDFunct;
        this.tSPDFunct = tSPDFunct;
        this.fieldOrientedFunct = () -> Constants.Drivetrain.kIsFieldOriented;
        this.xSlewLimit = new SlewRateLimiter(Constants.Drivetrain.kMaxAccel);
        this.ySlewLimit = new SlewRateLimiter(Constants.Drivetrain.kMaxAccel);
        this.tSlewLimit = new SlewRateLimiter(Constants.Drivetrain.kMaxTurnAccel);
        
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
    // Speed with deadband
    double xSpeed = runDeadband(xSPDFunct.get());
    double ySpeed = runDeadband(ySPDFunct.get());
    double tSpeed = runDeadband(tSPDFunct.get());

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("tSpeed", tSpeed);

    // Slew limit (accel)
    if (!Constants.isScoreConeAlign) {
      xSpeed = xSlewLimit.calculate(xSpeed) * Constants.Drivetrain.kMaxSpeed;
      ySpeed = ySlewLimit.calculate(ySpeed) * Constants.Drivetrain.kMaxSpeed;
      tSpeed = tSlewLimit.calculate(tSpeed) * Constants.Drivetrain.kMaxTurnSpeed;
    }
    
    SmartDashboard.putNumber("SLEW xSpeed", xSpeed);
    SmartDashboard.putNumber("SLEW ySpeed", ySpeed);
    SmartDashboard.putNumber("SLEW tSpeed", tSpeed);

    // Apply to modules
    if (Constants.isEnabled) {
      swerveDrive.setModuleSpeeds(xSpeed, ySpeed, tSpeed);
    }
    // SmartDashboard.putString("Module States Desaturated", moduleStates[2].toString());

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


