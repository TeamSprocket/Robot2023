// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macro;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunMotorPersistent extends CommandBase {
  boolean isFinished = false;
  WPI_TalonFX talon;
  double speed;
  /** Creates a new RunMotorPersistent. */
  public RunMotorPersistent(WPI_TalonFX talon, double speed) {
    this.talon = talon;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void stop() {
    isFinished = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AOIDFJAUOIFJESLGRSOILJELIFJAESLIGSEJLIAEJFLAIEJFLAIWFJLAIJFLIAWJFLIAWF");
    talon.set(ControlMode.PercentOutput, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    
    System.out.println("RUNNING");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
