// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.ZeroWheelsTEST;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.Directions;

public class RobotContainer {

  private final CommandXboxController driver = new CommandXboxController(0);
  SwerveDrive swerveDrive = new SwerveDrive();

  public RobotContainer() {
    configureBindings();
  }

  
  public void configureBindings() {
    // --------------------=Driver=--------------------
    driver.x().onTrue(new ZeroWheelsTEST(swerveDrive));

    driver.button(RobotMap.Controller.Y).onTrue(Commands.runOnce(() -> swerveDrive.setHeading(Directions.FORWARD), swerveDrive));
    driver.button(RobotMap.Controller.X).onTrue(Commands.runOnce(() -> swerveDrive.setHeading(Directions.LEFT), swerveDrive));
    driver.button(RobotMap.Controller.B).onTrue(Commands.runOnce(() -> swerveDrive.setHeading(Directions.RIGHT), swerveDrive));
    driver.button(RobotMap.Controller.A).onTrue(Commands.runOnce(() -> swerveDrive.setHeading(Directions.BACK), swerveDrive));
  }

  public Command getDriveTeleop() {
    return new DriveTeleop(
			swerveDrive, 
			() -> driver.getLeftX(), 
			() -> driver.getLeftY(), 
			() -> driver.getRightX());
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
