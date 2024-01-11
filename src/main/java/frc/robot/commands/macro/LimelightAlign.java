// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macro;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.LimelightSubsystem;


public class LimelightAlign extends CommandBase {

  private final LimelightSubsystem;

  public LimelightAlign(LimelightSubsystem limelightSubsystem) {
    this.LimelightSubsystem = limelightSubsystem;

    addRequirements(limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.isEnabled = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelightSubsystem.setSpeeds();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopModules();
    Constants.isEnabled = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
