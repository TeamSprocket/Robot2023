// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class PIDTurnTimed extends CommandBase {
  /** Creates a new PIDTurnTimed. */
  // ADIS16470_IMU imu;
  SwerveDrive swerveDrive;
  Timer timer;
  double duration, target;
  boolean isFinished = false;

  public PIDTurnTimed(SwerveDrive swerveDrive, double target, double duration) {
    // this.imu = imu;
    this.swerveDrive = swerveDrive;
    this.timer = new Timer();
    this.duration = duration;
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.start();
    double speed = 0;
    
    if (swerveDrive.getHeadingRad() <= target) {
      speed = 0.1;
    } else {
      speed = -0.1;
    }

    if (Math.abs(swerveDrive.getHeadingRad() - target) <= (Math.toRadians(5))) {
      isFinished = true;
    }
      

    ChassisSpeeds chassisSpeeds;
    // // Field Oriented
    if (Constants.Drivetrain.IS_FIELD_ORIENTED) {
      double headingRad = Math.toRadians(swerveDrive.getHeading());
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          0, 0, speed, new Rotation2d(headingRad));
    } else { 
      chassisSpeeds = new ChassisSpeeds(0, 0, speed);
    }

    SwerveModuleState[] moduleStates = Constants.Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    swerveDrive.setModuleStates(moduleStates);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= duration) || isFinished;
  }
}
