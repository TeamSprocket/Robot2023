// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class ResetGyro extends CommandBase {
  /** Creates a new ResetGyro. */
  public ResetGyro(SwerveDrive swerveDrive) {
    swerveDrive.zeroHeading();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
