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

public class BalanceOnChargeStation extends CommandBase {
  SwerveDrive swerveDrive;
  // ADIS16470_IMU gyro;
  boolean isFinished = false;
  double speedInitial;
  boolean onRamp = false;
  Timer timer;
  double waitTime = 2.2;
  // PIDController controller;
   
  /**
   * 
   * @param swerveDrive swerveDrive object
   * @param speedInitial initiall speed to approach charging station at, reduced to 0.05 if it surpasses it
   */
  public BalanceOnChargeStation(SwerveDrive swerveDrive, double speedInitial) {
    this.swerveDrive = swerveDrive;
    this.timer = new Timer();
    this.speedInitial = -1.0 * speedInitial;

    if (this.speedInitial >= 0.05) {
      this.speedInitial = 0.05;
    }
    if (this.speedInitial <= -0.05) {
      this.speedInitial = -0.05;
    }
    // this.gyro = new ADIS16470_IMU();
    
   
    // this.controller = new PIDController(Constants.Auton.kPBalance, Constants.Auton.kIBalance, Constants.Auton.kDBalance);
    
  /** Creates a new BalanceOnChargeStation. */
  // public BalanceOnChargeStation() {
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
    double angle = swerveDrive.getPitchDeg();
    double speed = speedInitial;

    if (angle > 10) {
      onRamp = true;
    }

    if (onRamp) {
      timer.start();

      // speed = 0.01;
    }

    if (onRamp && timer.get() < waitTime) {
      speed = Constants.Auton.SPEED_ON_RAMP * (Math.abs(speedInitial) / speedInitial);
    }

    if (timer.get() > waitTime) {
      speed = 0;
      lockWheels();
    }

    if (timer.get() > waitTime + 0.25) {
      isFinished = true;
    }
    

    setSpeeds(speed);
    
  }

  public void setSpeeds(double output) {
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

  public void lockWheels() {
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(0, 0.0001, 0);

    // Calculate module states per module
    SwerveModuleState[] moduleStates = Constants.Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);
  
    // Apply to modules
    swerveDrive.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isFinished) {
      return true;
    }
    return false;
  }
}
