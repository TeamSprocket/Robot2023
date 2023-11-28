// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.ZeroWheelsTEST;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {

  private final CommandXboxController driver = new CommandXboxController(0);
  SwerveDrive swerveDrive = new SwerveDrive();

  public RobotContainer() {
    configureBindings();
  }

  
  public void configureBindings() {
    // --------------------=Driver=--------------------
		swerveDrive.setDefaultCommand(new DriveTeleop(
			swerveDrive, 
			() -> driver.getLeftX(), 
			() -> -driver.getLeftY(), 
			() -> -driver.getRightX()));
    driver.x().onTrue(new ZeroWheelsTEST(swerveDrive));
  }
  
  public void resetModulesToAbsolute() {
    swerveDrive.resetModulesToAbsolute();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
