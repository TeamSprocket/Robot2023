// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDControlHelper extends CommandBase {
  private Timer timer = new Timer();
  private boolean stopped = false;
  private double min = Double.MIN_VALUE;
  private double max = Double.MAX_VALUE;
  private double duration = Double.MAX_VALUE;
  private double kP, kI, kD;

  private PIDController pidController;
  

  public PIDControlHelper(double kP, double kI, double kD) { 
    pidController = new PIDController(kP, kI, kD);
  }
  public PIDControlHelper(double kP, double kI, double kD, double min, double max) { 
    pidController = new PIDController(kP, kI, kD);
    this.min = min;
    this.max = max;
  }
  public PIDControlHelper(double kP, double kI, double kD, double min, double max, double duration) { 
    pidController = new PIDController(kP, kI, kD);
    this.min = min;
    this.max = max;
    this.duration = duration;
  }


  public void stop() {
    stopped = true;
  }
  public void setMin(double min) {
    this.min = min;
  }
  public void setMax(double max) {
    this.max = max;
  }
  public void setDuration(double duration) {
    this.duration = duration;
  }
  public void setPID(double kP, double kI, double kD) {
    pidController = new PIDController(kP, kI, kD);
  }


  public double getMax() {
      return max;
  }
  public double getMin() {
      return min;
  }
  public double getDuration() {
      return duration;
  }

  
  public double calculate(double current, double setpoint) {
    double val = pidController.calculate(current, setpoint);
    if (val < min) {
      val = min;
    } else if (val > max) {
      val = max;
    }
    return val;
  }


  @Override
  public void initialize() {
    timer.reset();
  }
  
  
  @Override
  public void execute() {
    timer.start();
  }

  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > duration || stopped;
    
  }
}
