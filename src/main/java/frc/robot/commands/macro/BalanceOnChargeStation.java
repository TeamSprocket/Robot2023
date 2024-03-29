// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macro;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.util.Util;

public class BalanceOnChargeStation extends CommandBase {
  SwerveDrive swerveDrive;
  double speedInitial;
  boolean onRamp = false;
  Timer timer = new Timer();
  Timer endTimer = new Timer();
  double duration;

  /**
   * 
   * @param swerveDrive swerveDrive object
   * @param speedInitial initiall speed to approach charging station at, reduced to 0.05 if it surpasses it
   */
  public BalanceOnChargeStation(SwerveDrive swerveDrive, double speedInitial, double duration) {
    this.swerveDrive = swerveDrive;
    this.speedInitial = speedInitial / 10.0;
    this.duration = duration;

    // Deadband (hard limit)
    Util.minmax(this.speedInitial, -0.2, 0.2);
  }

  @Override
  public void initialize() {
    timer.reset();
    endTimer.reset();
    
    swerveDrive.setTurnDefaultMode(NeutralMode.Brake);
    swerveDrive.setDriveDefaultMode(NeutralMode.Brake);
  }
  
  @Override
  public void execute() {
    timer.start();
    double speed = speedInitial;
    double turn = 0;

    
    if (Math.abs(swerveDrive.getPitchDeg()) >= (Constants.Auton.kChargeStationAngle - Constants.Auton.kOnChargeStationTolerance)) {
      onRamp = true;
    } 

    if (onRamp) {
        speed = Constants.Auton.kSpeedWhileClimbing;

      if (swerveDrive.getPitchDeg() >= (Constants.Auton.kChargeStationAngle - Constants.Auton.kChargeStationBalanceTolerance)) {
        endTimer.stop();
      } else { // Below tolerance
        endTimer.start();
      }
    }

    // Lock Wheels
    if (endTimer.get() >= Constants.Auton.BALANCE_END_TIME_THRESHOLD) {
      speed = 0;
      turn = 0.01;
    } 

    putDebugInfo(speed);
    setSpeeds(speed, turn);
    
  }
  
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopModules();
  }

  @Override
  public boolean isFinished() {
    return timer.get() > duration || endTimer.get() >= (Constants.Auton.BALANCE_END_TIME_THRESHOLD + 0.5);
    // return timer.get() > duration || endTimer.get() >= (Constants.Auton.BALANCE_END_TIME_THRESHOLD);
  }







  public void setSpeeds(double output, double turn) {
    double vYSpeed = -1 * output;

    ChassisSpeeds chassisSpeeds;
    if (Constants.Drivetrain.kIsFieldOriented) {
      double headingRad = Math.toRadians(swerveDrive.getHeading());
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        vYSpeed, 0, turn, new Rotation2d(headingRad));
    } else { 
      chassisSpeeds = new ChassisSpeeds(output, 0, 0);
    }

    // Calculate module states per module
    SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
  
    // Apply to modules
    swerveDrive.setModuleStates(moduleStates);
  }

  public void putDebugInfo(double speed) {
    SmartDashboard.putBoolean("On Ramp", onRamp);
    SmartDashboard.putNumber("Balance drive spd", speed);

  }




  

}
