// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macro.timed;

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


public class LimelightAlignTimed extends CommandBase {
  NetworkTable table;
  double tValid = 1;
  double tXOffset;
  // Timer timer;
  double verticalSpeed;
  SwerveDrive swerveDrive;
  PIDController pidController;
  double duration;
  Timer timer;
  // NetworkTableEntry tValid, tXOffset, tYOffset, tArea;
  /** Creates a new LimelightAlign. */
  public LimelightAlignTimed(SwerveDrive swerveDrive, double duration) {
    this.timer = new Timer();
    this.swerveDrive = swerveDrive;
    this.duration = duration;
    this.verticalSpeed = 0.0;
    // this.duration = duration;

    timer = new Timer();
    this.pidController = new PIDController(Constants.Drivetrain.kLimelightP, Constants.Drivetrain.kLimelightI, Constants.Drivetrain.kLimelightD);
    this.pidController.setSetpoint(0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public LimelightAlignTimed(SwerveDrive swerveDrive, double verticalSpeed, double duration) {
    this.timer = new Timer();
    this.swerveDrive = swerveDrive;
    this.duration = duration;
    this.verticalSpeed = -verticalSpeed;
    // this.duration = duration;

    timer = new Timer();
    this.pidController = new PIDController(Constants.Drivetrain.kLimelightP, Constants.Drivetrain.kLimelightI, Constants.Drivetrain.kLimelightD);
    this.pidController.setSetpoint(0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    // timer.reset();
    Constants.Drivetrain.JOYSTICK_DRIVING_ENABLED = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.start();
    System.out.println("ALIGNING");

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tValid = table.getEntry("tv").getDouble(0.0); // 0 = invalid, 1 = valid
    tXOffset = table.getEntry("tx").getDouble(0.0); // x offset -27 to 27 deg
    // double tYOffset = table.getEntry("ty").getDouble(0.0); // y offset -20.5 to 20.5 deg
    // double tArea = table.getEntry("ty").getDouble(0.0); // area of target in img (percent of total img)
    double output = -1 * pidController.calculate(tXOffset);
    setSpeeds(output);

  }



  public void setSpeeds(double output) {
    ChassisSpeeds chassisSpeeds;
    if (Constants.Drivetrain.IS_FIELD_ORIENTED) {
      double headingRad = Math.toRadians(swerveDrive.getHeading());
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          verticalSpeed, output, 0, new Rotation2d(headingRad));
    } else { 
      chassisSpeeds = new ChassisSpeeds(verticalSpeed, output, 0);
    }

    // Calculate module states per module
    SwerveModuleState[] moduleStates = Constants.Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);
  
    // Apply to modules
    swerveDrive.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopModules();
    Constants.Drivetrain.JOYSTICK_DRIVING_ENABLED = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > duration;
    // return true;
  }

  




  


}
