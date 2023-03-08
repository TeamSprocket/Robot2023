// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macro;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ShootClaw extends CommandBase {
  /** Creates a new ShootClaw. */
  double speed;
  PIDController pidController;
  WPI_TalonFX talonLeft, talonRight; 
  double currentRPS, output;

  public ShootClaw(double speed) {
    this.speed = speed;
    this.pidController = new PIDController(Constants.Claw.kPShooter, Constants.Claw.kIShooter, Constants.Claw.kDShooter);
    talonLeft = new WPI_TalonFX(RobotMap.Claw.CLAW_LEFT);
    talonRight = new WPI_TalonFX(RobotMap.Claw.CLAW_RIGHT);

    talonLeft.setInverted(true);

    // talonRight.follow(talonLeft);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentRPS = talonLeft.getSelectedSensorVelocity() * 10.0 / 2048.0 / 2.0;
    output = pidController.calculate(currentRPS / 3.5041, speed);
    talonLeft.set(ControlMode.PercentOutput, output);
    talonRight.set(ControlMode.PercentOutput, output);
    System.out.println(output);

    SmartDashboard.putNumber("Shooter RPS", currentRPS);
    SmartDashboard.putNumber("Shooter Output", output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
