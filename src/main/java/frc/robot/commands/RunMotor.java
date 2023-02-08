// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.util.commands.PersistentCommand;

public class RunMotor extends PersistentCommand {
  private final XboxController controller;
  private WPI_TalonFX talon;

  public RunMotor(XboxController controller, int talonNum) {
    this.controller = controller;
    talon = new WPI_TalonFX(talonNum);
  }

  @Override
  public void execute() {
    Double speed = controller.getLeftY();
    if (speed < 0.01) 
      speed = 0.0;
    
    talon.set(speed);
    SmartDashboard.putNumber("TalonSpeed", speed);

    

    // double left = Util.deadband(0.1, leftJoystick.getY());
    // double right = Util.deadband(0.1, rightJoystick.getY());
    // drivetrain.tankDrive(left, right);
  }

  @Override
  public void end(boolean interrupted) {
    talon.set(0);
  }

}