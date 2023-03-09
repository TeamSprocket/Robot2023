// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class OneMeterForward extends CommandBase {
  /** Creates a new OneMeterForward. */
  SwerveDrive swerveDrive;
  PIDController controller;
  
  public OneMeterForward(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;

    this.controller = new PIDController(Constants.Auton.kPBalance, Constants.Auton.kIBalance, Constants.Auton.kDBalance);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = controller.calculate(swerveDrive.getDrivePosition(), 1);
    if (output >= 0.1) {
      output = 0.1;
    }

    ChassisSpeeds chassisSpeeds;
    if (Constants.Drivetrain.IS_FIELD_ORIENTED) {
      double headingRad = Math.toRadians(swerveDrive.getHeading());
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          output, 0, 0, new Rotation2d(headingRad));
    } else { 
      chassisSpeeds = new ChassisSpeeds(output, 0, 0);
    }

    // Calculate module states per module
    SwerveModuleState[] moduleStates = Constants.Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);
  
    // Apply to modules
    swerveDrive.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double offset = Math.abs(1 - swerveDrive.getDrivePosition());
    if (offset < 0.1) {
      return true;
    }
    return false;
  }
}
