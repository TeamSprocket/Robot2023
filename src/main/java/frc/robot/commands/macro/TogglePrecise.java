// Copyright (c) FIRST and other WPILib contributors.

package frc.robot.commands.macro;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class TogglePrecise extends InstantCommand {

  SwerveDrive swerveDrive;
  
  public TogglePrecise(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  @Override
  public void initialize() {
    Constants.Drivetrain.IS_PRECISE = !(Constants.Drivetrain.IS_PRECISE);
  }
}
