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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class BalanceOnChargeStation extends CommandBase {
  SwerveDrive swerveDrive;
  // ADIS16470_IMU gyro;
  boolean isFinished = false;
  double speedInitial;
  boolean onRamp = false;
  Timer timer, rampTimer;
  double waitTime = 2;
  Timer endTimer, onRampTimer;

  double onRampAngle = Constants.Auton.kChargeStationAngle;

  double duration;
  // PIDController controller;
   
  /**
   * 
   * @param swerveDrive swerveDrive object
   * @param speedInitial initiall speed to approach charging station at, reduced to 0.05 if it surpasses it
   */
  public BalanceOnChargeStation(SwerveDrive swerveDrive, double speedInitial, boolean climbFromBackOfBot, double duration) {
    this.swerveDrive = swerveDrive;
    this.timer = new Timer();
    this.endTimer = new Timer();
    this.onRampTimer = new Timer();
    this.speedInitial = -1.0 * speedInitial;
    this.duration = duration;

    if (!climbFromBackOfBot) {
      this.speedInitial *= -1;
      this.onRampAngle *= -1;
    }

    if (this.speedInitial >= 0.1) {
      this.speedInitial = 0.1;
    }
    if (this.speedInitial <= -0.1) {
      this.speedInitial = -0.1;
    }
    // this.gyro = new ADIS16470_IMU();
  /** Creates a new BalanceOnChargeStation. */
  // public BalanceOnChargeStation() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    endTimer.reset();
    onRampTimer.reset();
    // swerveDrive.zeroPitch();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.setTurnDefaultMode(NeutralMode.Brake);
    swerveDrive.setDriveDefaultMode(NeutralMode.Brake);

    timer.start();

    double speed = speedInitial;
    double turn = 0;

    if (Math.abs(swerveDrive.getPitchDeg()) >= (onRampAngle - Constants.Auton.kOnChargeStationTolerance)) {
      onRamp = true;
    }

    if (onRamp) {
      if (swerveDrive.getPitchDeg() >= (onRampAngle - Constants.Auton.kChargeStationBalanceTolerance)) { //climbing
        speed = Constants.Auton.kSpeedWhileClimbing;
      } else { //falling
        speed = 0;
        turn = 0.01;
        endTimer.start();
      }
    }

    setSpeeds(speed, turn);
    
  }

  public void setSpeeds(double output, double turn) {
    ChassisSpeeds chassisSpeeds;
    if (Constants.Drivetrain.kIsFieldOriented) {
      double headingRad = Math.toRadians(swerveDrive.getHeading());
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          output, 0, turn, new Rotation2d(headingRad));
    } else { 
      chassisSpeeds = new ChassisSpeeds(output, 0, 0);
    }

    // Calculate module states per module
    SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
  
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
    return timer.get() > duration || endTimer.get() >= Constants.Auton.BALANCE_END_TIME_THRESHOLD;
    
  }
}
