package frc.robot.commands.macro.timed;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auton;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.Constants;
// import frc.robot.subsystems.SwerveDrive;

// public class PIDDriveTimed extends CommandBase {
//   /** Creates a new OneMeterForward. */
//   SwerveDrive swerveDrive;
//   PIDController controller;
//   double duration, distanceV, distanceH;
//   Timer timer;
//   // SlewRateLimiter slewLimiter;

//   double sign;
  
//   public PIDDriveTimed(SwerveDrive swerveDrive, double distanceV, double distanceH, double duration) {
//     this.swerveDrive = swerveDrive;
//     this.duration = duration;
//     this.distanceV = distanceV;
//     // this.distanceH = -distanceH;
//     // double sign = (Math.abs(this.distanceV) / (this.distanceV + 0.000001));

//     this.timer = new Timer();

//     // this.slewLimiter = new SlewRateLimiter(0.01);

//     this.controller = new PIDController(0.02, 0, 0.00);
//     // this.controller = new PIDController(0.0, Constants.Auton.kIBalance, Constants.Auton.kDBalance);
//     controller.setSetpoint(this.distanceV); 
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer.reset();
//     swerveDrive.zeroDrive();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     timer.start();

//     double output = controller.calculate(swerveDrive.getDrivePosition());
//     // output = slewLimiter.calculate(output);
//     System.out.println(output);
    
//     // if (output >= 0.02) {
//       // output = 0.02;
//     // }
//     // if (output <= -0.02) {
//       // output = -0.02;
//     // }

//     ChassisSpeeds chassisSpeeds;
//     if (Constants.Drivetrain.IS_FIELD_ORIENTED) {
//       double headingRad = Math.toRadians(swerveDrive.getHeading());
//       chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//           output, 0, 0, new Rotation2d(headingRad));
//     } else { 
//       chassisSpeeds = new ChassisSpeeds(output, 0, 0);
//     }

//     // Calculate module states per module
//     SwerveModuleState[] moduleStates = Constants.Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);
  
//     // Apply to modules
//     swerveDrive.setModuleStates(moduleStates);

//     SmartDashboard.putNumber("Drive Position", swerveDrive.getDrivePosition());
//     SmartDashboard.putNumber("Drive Position Output PID", output);
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
//     // double offset = Math.abs(1 - swerveDrive.getDrivePosition());
//     // if (offset < 0.01) {
//     //   return true;
//     // }
//     // return false;
//   }
// }
