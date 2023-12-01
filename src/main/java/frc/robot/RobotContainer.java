// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeConeCmd;
import frc.robot.commands.IntakeCubeCmd;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  // controllers
  public static XboxController controller = new XboxController(Constants.controller.controllerPort);
  
  // subsystems
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // commands
  public static IntakeConeCmd intakeConeCmd = new IntakeConeCmd(intakeSubsystem);
  public static IntakeCubeCmd intakeCubeCmd = new IntakeCubeCmd(intakeSubsystem);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(controller, Constants.controller.LT).whileTrue(intakeConeCmd);
    new JoystickButton(controller, Constants.controller.RT).whileTrue(intakeCubeCmd);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}