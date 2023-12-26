// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

/** Creates a new Intake. */
  private final WPI_TalonFX motor = new WPI_TalonFX(Constants.IntakeConst.motorPort);

  public boolean containCone = false;
  public boolean containCube = false;

  private int direction = 1;
  
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Current", motor.getStatorCurrent());
    // This method will be called once per scheduler run
  }

  public void intakeIdle() {
    motor.set(direction * Constants.IntakeConst.idleSpeed);
    // take "direction" of the motor and multiply by idle speed to change direction
  }

  public void intakeCone() {

    if (containCone == false) { 
      direction = 1;
      motor.set(Constants.IntakeConst.intakeconeSpeed);

      // if (getCurrent() > Constants.IntakeConst.currentThreshold) {

      //   if (RobotContainer.controller.getRightBumperReleased()) {
      //     containCone = true;
      //   }
      // }
    }
    else {
      direction = -1;
      motor.set(Constants.IntakeConst.outtakeconeSpeed);

      // if (RobotContainer.controller.getRightBumperReleased()) {
      //   containCone = false;
      // }
    }
  }

  public void intakeCube() {
    
    if (containCube == false) { 
      direction = -1;
      motor.set(Constants.IntakeConst.intakecubeSpeed);

      // if (getCurrent() > Constants.IntakeConst.currentThreshold) {

      //   if (RobotContainer.controller.getLeftBumperReleased()) {
      //     containCube = true;
      //   }
      // }
    }
    else {
      direction = 1;
      motor.set(Constants.IntakeConst.outtakecubeSpeed);

      // if (RobotContainer.controller.getLeftBumperReleased()) {
      //   containCube = false;
      // }
    }
  }

  public double getCurrent() {
    double current = motor.getStatorCurrent();
    // return current to check if we have an object in the intake
    return current;
  }

  public void stopIntake() {
    motor.set(0.0);
  }
}
