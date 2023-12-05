// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final WPI_TalonFX motor1 = new WPI_TalonFX(0);
  private final WPI_TalonFX motor2= new WPI_TalonFX(0);
  /** Creates a new elevator. */
  public Elevator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void goingup(){
    motor1.set(1);
    motor2.set(1);
  }
  public void goingdown(){
    motor1.set(-1);
    motor2.set(-1);
  }
  public void stop(){
    motor1.set(0);
    motor2.set(0);
  }

}
