// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macro;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.Direction;

public class SwerveUtils extends CommandBase {
  public enum SwerveUtilsCommands {
    ZERO_HEADING,
    TOGGLE_PRECISE,
    UPDATE_HEADING_FRONT,
    UPDATE_HEADING_LEFT,
    UPDATE_HEADING_RIGHT,
    UPDATE_HEADING_BACK
  }

  SwerveDrive swerveDrive;
  SwerveUtilsCommands command;

  public SwerveUtils(SwerveDrive swerveDrive, SwerveUtilsCommands command) {
    this.swerveDrive = swerveDrive;
    this.command = command;
  }

  @Override
  public void initialize() {
    switch (command) {
      case ZERO_HEADING: 
        swerveDrive.zeroHeading();
        break;
      case TOGGLE_PRECISE: 
        swerveDrive.togglePrecise();
        break;
      case UPDATE_HEADING_FRONT: 
        swerveDrive.updateHeading(Direction.FRONT);
        break;
      case UPDATE_HEADING_LEFT: 
        swerveDrive.updateHeading(Direction.LEFT);
        break;
      case UPDATE_HEADING_RIGHT: 
        swerveDrive.updateHeading(Direction.RIGHT);
        break;
      case UPDATE_HEADING_BACK: 
        swerveDrive.updateHeading(Direction.BACK);
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
