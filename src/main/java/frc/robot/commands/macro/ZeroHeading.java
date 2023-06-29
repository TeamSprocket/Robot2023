// Copyright (c) FIRST and other WPILib contributors.

package frc.robot.commands.macro;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrive;

public class ZeroHeading extends InstantCommand {

  SwerveDrive swerveDrive;
  
  public ZeroHeading(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  @Override
  public void initialize() {
    swerveDrive.zeroHeading();
  }
}
