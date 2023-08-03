
package frc.util;

import edu.wpi.first.math.controller.PIDController;


// Improved PID Controller

public class PIDPlus {
  PIDController controller;
  double ff;
  double min, max;
  double minValueDeadband, maxValueDeadband;

  public PIDPlus(double p, double i, double d) {
    controller = new PIDController(p, i, d);
    this.ff = 0;
  }

  public PIDPlus(double p, double i, double d, double ff) {
    this.ff = ff;

    controller = new PIDController(p, i, d);
  }



  public double getSetpoint() {
    return controller.getSetpoint();
  }



  public void setSetpoint(double setpoint) {
    controller.setSetpoint(setpoint);
  }

  public void setDeadband(double minValueDeadband, double maxValueDeadband) {
    this.minValueDeadband = minValueDeadband;
    this.maxValueDeadband = maxValueDeadband;
  }

  public void setMin(double min) {
      this.min = min;
  }

  public void setMax(double max) {
      this.max = max;
  }  

  public void setMinMax(double min, double max) {
    this.min = min;
    this.max = max;
  }

  public double calculate(double current, double setpoint) {
    double value = controller.calculate(current, setpoint);

    value += ff;
    
    if (value >= minValueDeadband || value <= maxValueDeadband) 
      value = 0;

    if (value > max)
      value = max;
    if (value < min)
      value = min;
    
    return value;
  } 

  public double calculate(double current) {
    return calculate(current, controller.getSetpoint());
  }


}
