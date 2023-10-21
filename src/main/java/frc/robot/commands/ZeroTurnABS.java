// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class ZeroTurnABS extends CommandBase {
  SwerveDrive swerveDrive;
  Timer timer = new Timer();
  double resetTicks;

  public ZeroTurnABS(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // timer.reset();
    // resetTicks = swerveDrive.getTurnResetTicks();
    swerveDrive.resetTurnABS();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // timer.start();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // swerveDrive.zeroTurnABS();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return timer.get() > 0.1;
    return true;
  }
}
