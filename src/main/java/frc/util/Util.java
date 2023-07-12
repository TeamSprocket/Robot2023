/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

/**
 * This class contains miscellaneous utility functions that can be used anywhere
 * in the robot project.
 */
public final class Util {
  /**
   * Return 0 if the value is within -range..range. Otherwise return the value.
   */
  public static double deadband(double range, double value) {
    if(value < range && value > -range)
      return 0;
    else
      return value;
  }
  
  /**
   * Corrects an input based on a polynomial function.
   */
  public static double polynomialCorrect(double input, double... coefficients) {
    double output = 0;
    double term = 1;

    for(double c : coefficients) {
      output += c * term;
      term *= input;
    }

    return output;
  }

  public static double scale(double val, double fromLow, double fromHigh, double toLow, double toHigh) {
    double normalizedVal = (val - fromLow) / (fromHigh - fromLow);
    return normalizedVal * (toHigh - toLow) + toLow;
  }

  public static double From360To180(double deg) {
    if ((deg % 360.0) > 180.0) {
      return (deg % 360.0) - 360.0;
    }
    return (deg % 360.0);
  }

  public static double From180To360(double deg) {
    if ((deg % 360.0) < 0.0) {
      return (deg % 360.0) + 360.0;
    }
    return (deg % 360.0);
  }

  public static double From2PiToPi(double deg) {
    if ((deg % (2 * Math.PI)) > Math.PI) {
      return (deg % (2 * Math.PI)) - (2 * Math.PI);
    }
    return (deg % (2 * Math.PI));
  }

  public static double FromPiTo2Pi(double deg) {
    if ((deg % (2 * Math.PI)) < 0.0) {
      return (deg % (2 * Math.PI)) + (2 * Math.PI);
    }
    return (deg % (2 * Math.PI));
  }



}
