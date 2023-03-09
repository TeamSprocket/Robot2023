
package frc.robot.commands.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.SwerveDrive;

public class BalanceOnChargeStationVertical extends CommandBase {

  private final PIDController controller; 
  private final SwerveDrive swerveDrive;
  private final Timer timer;
  private final double duration;
  private double loc, baseTicks;
  
  // private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  /** Creates a new BalanceOnChargeStation. */
  public BalanceOnChargeStationVertical(SwerveDrive swerveDrive, double duration) {
    controller = new PIDController(Constants.Auton.kPBalance, Constants.Auton.kIBalance, Constants.Auton.kDBalance);
    this.swerveDrive = swerveDrive;
    timer = new Timer();
    this.duration = duration;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    this.loc = 0;
    this.baseTicks = swerveDrive.getFrontLeftTicks();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.start();
    loc = getLocation(loc);

    double zeroingNum = Constants.Auton.BUMPER_THICKNESS_Y_METERS + Constants.Auton.CENTER_OF_MASS_FROMT_BACK_Y_METERS;
    double setpoint = -1 * (zeroingNum + Constants.Auton.CHARGING_STATION_TO_CENTER_Y_METERS);
    
    double output = controller.calculate(loc, setpoint);

    applySpeeds(output);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() >= duration) {
      return true;
    }
    return false;
  }


  public void applySpeeds(double spd) {
    double speed = spd;
    if (speed >= Constants.Drivetrain.kMaxSpeedMetersPerSecond / 1000000) {
      speed = Constants.Drivetrain.kMaxSpeedMetersPerSecond / 1000000;
    }

    ChassisSpeeds chassisSpeeds;

    if (Constants.Drivetrain.IS_FIELD_ORIENTED) {
    double headingRad = Math.toRadians(swerveDrive.getHeading());
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        speed, 0.0, 0.0, new Rotation2d(headingRad));
    } else { 
        chassisSpeeds = new ChassisSpeeds(speed, 0.0, 0.0);
    }

    // Calculate module states per module
    SwerveModuleState[] moduleStates = Constants.Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    // Apply to modules
    swerveDrive.setModuleStates(moduleStates);
}

    public double getLocation(double location) {
        double loc = location;
        double absTickOffset = Math.abs(baseTicks - swerveDrive.getFrontLeftTicks());
        double ticksPerWheelRot = 2048.0 * Constants.Drivetrain.kDriveMotorGearRatio;
        double wheelPosPercent = absTickOffset / ticksPerWheelRot;
        double wheelCirc =  2 * Math.PI * (Constants.Drivetrain.kWheelDiameterMeters / 2);
        loc += (wheelPosPercent * wheelCirc);
        return loc;
    }














}



