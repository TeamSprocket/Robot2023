
// package frc.robot.commands.persistent;

// import java.util.function.Supplier;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.SwerveDrive;
// import frc.robot.subsystems.SwerveModule;

// public class DriveTimed extends CommandBase {

//   private final Timer timer;
//   private final SwerveDrive swerveDrive;
//   private final double xSpeed, ySpeed, tSpeed;
//   private final double xSlewLimit, ySlewLimit, tSlewLimit;
//   private final double duration;

//   /**
//    * Used to send outputs to SwerveDrive
//    * @param swerveDrive SwerveDrive object
//    * @param xSPDFunct Vertical speed of the bot
//    * @param ySPDFunct Horizontal speed of the bot
//    * @param tSPDFunct Angular speed of the bot 
//    */
//   public DriveTimed(SwerveDrive swerveDrive, double xSpeed, double ySpeed, double tSpeed, double duration) {
//       this.swerveDrive = swerveDrive;
//       this.xSpeed = xSpeed;
//       this.ySpeed = ySpeed;
//       this.tSpeed = tSpeed;
//       this.xSlewLimit = Constants.Auton.kMaxAccelPerSec;
//       this.ySlewLimit = Constants.Auton.kMaxAccelPerSec;
//       this.tSlewLimit = Constants.Auton.kMaxAccelAngularPerSec;

//       this.duration = duration;

//       timer = new Timer();
//   }

//   @Override
//   public void initialize() {
//     timer.reset();
//   }

//   @Override
//   public void execute() {
//     timer.start();

//     xSpeed = xSlewLimit.calculate(xSpeed);
//     ySpeed = ySlewLimit.calculate(ySpeed);
//     tSpeed = tSlewLimit.calculate(tSpeed);

//     ChassisSpeeds chassisSpeeds;
//     // Field Oriented
//     if (Constants.Drivetrain.IS_FIELD_ORIENTED) {
//       double headingRad = Math.toRadians(-swerveDrive.getHeading());
      
//       if (Constants.Auton.FACING_DRIVERS) {
//         headingRad += Math.PI;
//       }

//       chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//           xSpeed, ySpeed, tSpeed, new Rotation2d(headingRad));
//     } else { // Robot oriented
//       chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, tSpeed);
//       SmartDashboard.putString("Chassis Speed", chassisSpeeds.toString());

//     }

//     // Calculate module states per module
//     SwerveModuleState[] moduleStates = Constants.Measurements.driveKinematics.toSwerveModuleStates(chassisSpeeds);
//     SmartDashboard.putString("Module States", moduleStates[2].toString());

//     // Normalize speeds
//     SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.kMaxSpeed);

//     // Apply to modules
//     if (Constants.kIsTeleop) {
//       swerveDrive.setModuleStates(moduleStates);
//     }
//     SmartDashboard.putString("Module States Desaturated", moduleStates[2].toString());

//   }

//   @Override
//   public void end(boolean interrupted) {
//     swerveDrive.stopModules();
//   }

//   @Override
//   public boolean isFinished() {
//     return timer.get() > duration;
//   }


//   public double calculateSlewManual(double value, double time, double limit) {
    
//   }


// }


