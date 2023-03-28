
package frc.robot.commands.macro.timed;

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
import frc.robot.commands.persistent.SwerveDriveCmd;
import frc.robot.subsystems.SwerveDrive;
import frc.util.commands.MacroCommand;

public class SwerveDriveCmdTimed extends MacroCommand {

  private final SwerveDrive swerveDrive;
  private final double yTarget, xTarget, tTarget;
  private final Timer timer;
  private final double duration;
  private boolean first = true;
  private double startTime;
  private WPI_TalonFX talonFL;
  private WPI_TalonFX talonFR;
  private WPI_TalonFX talonBL;
  private WPI_TalonFX talonBR;

  SlewRateLimiter xSlewLimit, ySlewLimit, tSlewLimit;

  // private RunMotorPersistent runMotorPersistentFL, runMotorPersistentFR, runMotorPersistentBL, runMotorPersistentBR;

  double xSpeed;
  double ySpeed;
  double tSpeed;

  /**
   * 
   * @param swerveDrive A SwerveDrive object
   * @param target Target Pose2d for the bot (given as x axis velocity from -1 to 1, y axis velocity from -1 to 1, and angular velocity in rad)
   * @param duration The number of seconds to traverse towards this target 
   * 
   */
  public SwerveDriveCmdTimed(SwerveDrive swerveDrive, Pose2d target, double duration) {
    this.swerveDrive = swerveDrive;
    this.xTarget = -1.0 * target.getY();
    this.yTarget = -1.0 * target.getX();
    this.tTarget = (target.getRotation().getRadians() / 10.0);
    this.timer = new Timer();
    this.duration = duration;
    talonFL = new WPI_TalonFX(RobotMap.Drivetrain.FRONT_LEFT_TALON_D);
    talonFR = new WPI_TalonFX(RobotMap.Drivetrain.FRONT_RIGHT_TALON_D);
    talonBL = new WPI_TalonFX(RobotMap.Drivetrain.BACK_LEFT_TALON_D);
    talonBR = new WPI_TalonFX(RobotMap.Drivetrain.BACK_RIGHT_TALON_D);

    // xSlewLimit = new SlewRateLimiter(Constants.Drivetrain.kTeleDriveMaxAccelerationUnitsPerSecond);
    // ySlewLimit = new SlewRateLimiter(Constants.Drivetrain.kTeleDriveMaxAccelerationUnitsPerSecond);
    // tSlewLimit = new SlewRateLimiter(Constants.Drivetrain.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    

    this.xSpeed = xTarget / 12;
    this.ySpeed = yTarget / 12;
    this.tSpeed = tTarget;
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

    // public double calculateSpeed() {
    //     double speed = 0;
    //   if (Math.abs(xSpeed) > Math.abs(ySpeed)) {
    //     speed = xSpeed;
    //   }
    //   else if (Math.abs(ySpeed) > Math.abs(xSpeed)) {
    //     speed = ySpeed;
    //   }
    //   else if (Math.abs(xSpeed) == Math.abs(ySpeed) && Math.abs(xSpeed) != 0.0) {
    //     speed = xSpeed;
    //   }
      
    //   if (Math.abs(speed) > Constants.Drivetrain.kMaxSpeedMetersPerSecond) {
    //     speed = Constants.Drivetrain.kMaxSpeedMetersPerSecond;
    //  }

    //  return speed / 4;
    // // runMotorPersistentFL = new RunMotorPersistent(talonFL, speed);
    // // runMotorPersistentFR = new RunMotorPersistent(talonFR, speed);
    // // runMotorPersistentBL = new RunMotorPersistent(talonBL, speed);
    // // runMotorPersistentBR = new RunMotorPersistent(talonBR, speed);
    
    // }

    
      
  // Called when the command is initially scheduled
  @Override
  public void initialize() {
    timer.reset();

    System.out.println("afoglisjLESGJSLIFEJLIAWJFLIGSJLEK>JFMLEKJFMLAEKFJMALIJBOILSWJFLKAWJFDLIAWJLIAWJFLWAIFJALWIFJWLKFJALWIFJA>WKFJLAWIFJALWIFJLAIWFJLAWIJFLAKSJFLIFJALKWJFLIGGNOIEJFLESNVIJNELIFJGNEKGJELKNKGDJFLISJLIAWJFLKAJKF");
    // new SwerveDriveCmd(
    //   swerveDrive, 
    //   () -> -0.5, 
    //   () -> 0.0, 
    //   () -> 0.0
    // );
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (first) {
    timer.start();

    // xSpeed = calculateSpeed();
    // ySpeed = xSpeed;

    // Slew limit (accel)
    // xSpeed = xSlewLimit.calculate(xSpeed) * Constants.Drivetrain.kMaxSpeedMetersPerSecond;
    // ySpeed = ySlewLimit.calculate(ySpeed) * Constants.Drivetrain.kMaxSpeedMetersPerSecond;
    // tSpeed = tSlewLimit.calculate(tSpeed) * Constants.Drivetrain.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    
      // System.out.println("SLEWWWW: " + xSpeed);

    // System.out.println("X CURRENT: " + xSpeed);
    

    ChassisSpeeds chassisSpeeds;
    // // Field Oriented
    if (Constants.Drivetrain.IS_FIELD_ORIENTED) {
      double headingRad = Math.toRadians(swerveDrive.getHeading());
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, tSpeed, new Rotation2d(headingRad));
    } else { // Robot oriented
      // xSpeed /= 10;
      // ySpee




      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, tSpeed);
      // SmartDashboard.putString("Chassis Speed", chassisSpeeds.toString());
    }

    // Calculate module states per module
    SwerveModuleState[] moduleStates = Constants.Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);
    
      

    // Normalize speeds
    // SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.kMaxSpeedMetersPerSecond);

      // System.out.println("Desaturated: " + chassisSpeeds.toString());

    // Apply to modules
    swerveDrive.setModuleStates(moduleStates);

    // driveMotors();

    
    
    
    
   

    
    
      // first = false;
    // }
    
    // new SwerveDriveCmd(
    //   swerveDrive, 
    //   () -> yTarget, 
    //   () -> xTarget, 
    //   () -> tTarget
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
    // runMotorPersistentFL.stop();
    // runMotorPersistentFR.stop();
    // runMotorPersistentBL.stop();
    // runMotorPersistentBR.stop();
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
