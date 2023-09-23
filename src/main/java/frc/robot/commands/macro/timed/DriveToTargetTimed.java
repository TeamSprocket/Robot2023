
// package frc.robot.commands.macro.timed;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.RobotPos;
// import frc.robot.subsystems.SwerveDrive;

// public class DriveToTargetTimed extends CommandBase {
//   double startingPosXMeters, startingPosYMeters, startingPosTRad;
//   double targetPosXMeters, targetPosYMeters, targetPosTRad;

//   PIDController xController, yController, tController;
//   Timer timer;

//   SwerveDrive swerveDrive;

//   SwerveDriveOdometry odometer;

//   double duration;

//   /**
//    * 
//    * @param swerveDrive SwerveDrive object
//    * @param targetPos Target pos of the bot, x and y in meters (left is -, right is +/ up is +, down is -). and target rotation in degrees
//    */
//   public DriveToTargetTimed(SwerveDrive swerveDrive, Pose2d targetPos, double duration) {
//     this.targetPosXMeters = targetPos.getX();
//     this.targetPosYMeters = targetPos.getY();
//     this.targetPosTRad = targetPos.getRotation().getRadians();

//     this.duration = duration;

//     xController = new PIDController(Constants.Auton.kPDriveToTargetXY, Constants.Auton.kIDriveToTargetXY, Constants.Auton.kDDriveToTargetXY);
//     yController = new PIDController(Constants.Auton.kPDriveToTargetXY, Constants.Auton.kIDriveToTargetXY, Constants.Auton.kDDriveToTargetXY);
//     tController = new PIDController(Constants.Auton.kPDriveToTargetT, Constants.Auton.kIDriveToTargetT, Constants.Auton.kDDriveToTargetT);
    
//     xController.setSetpoint(targetPosXMeters);
//     yController.setSetpoint(targetPosYMeters);
//     tController.setSetpoint(targetPosTRad);
    
//     timer = new Timer();

//     this.swerveDrive = swerveDrive;

//     SwerveModulePosition[] modulePositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
//     this.odometer = new SwerveDriveOdometry(Constants.Drivetrain.kDriveKinematics,
//       new Rotation2d(Math.PI),
//       modulePositions);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer.reset();

//     this.startingPosXMeters = RobotPos.getPos().getX();
//     this.startingPosYMeters = RobotPos.getPos().getY();
//     this.startingPosTRad = swerveDrive.getHeadingRad(); // 0 to 2 PI

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     timer.start();
//     odometer.update(new Rotation2d(swerveDrive.getHeadingRad()), swerveDrive.getSwerveModulePositions());
    
//     Pose2d currentPos = odometer.getPoseMeters();
//     double xOutput = xController.calculate(currentPos.getX());
//     double yOutput = xController.calculate(currentPos.getY());
//     double tOutput = xController.calculate(currentPos.getRotation().getRadians());
    
//     setModuleStates(xOutput, yOutput, tOutput, timer.get());
    
//   }

//   public void setModuleStates(double xSpeedIn, double ySpeedIn, double tSpeedIn, double time) {
//     double xSpeed = xSpeedIn;
//     double ySpeed = ySpeedIn;
//     double tSpeed = tSpeedIn;

//     if (Math.abs(xSpeed) > 0.05) {
//       xSpeed = 0.05 * (Math.abs(xSpeed + 0.00000001) / (0.00000001 + xSpeed));
//     } 
//     if (Math.abs(ySpeed) > 0.05) {
//       ySpeed = 0.05 * (Math.abs(ySpeed + 0.00000001) / (0.00000001 + ySpeed));
//     } 
//     if (Math.abs(tSpeed) > 0.05) {
//       tSpeed = 0.05 * (Math.abs(tSpeed + 0.00000001) / (0.00000001 + tSpeed));
//     } 

//     if (time <= 0.5) {
//       xSpeed *= (time * 2);
//       ySpeed *= (time * 2);
//       tSpeed *= (time * 2);
//     }

//     ChassisSpeeds chassisSpeeds;
//     double headingRad = Math.toRadians(swerveDrive.getHeading());
//     chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//         ySpeed, xSpeed, tSpeed, new Rotation2d(headingRad));
//     SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
//     swerveDrive.setModuleStates(moduleStates);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     swerveDrive.stopModules();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return timer.get() > duration;
//   }
// }
