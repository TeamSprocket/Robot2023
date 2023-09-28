// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macro;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class TogglePrecise extends CommandBase {
  SwerveDrive swerveDrive;
  public TogglePrecise(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  @Override
  public void initialize() {
    swerveDrive.togglePrecise();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
