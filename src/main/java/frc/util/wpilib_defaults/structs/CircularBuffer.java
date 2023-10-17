/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util.wpilib_defaults.structs;

/**
 * Add your docs here.
 */
public class CircularBuffer {
  private final double[] circularBuffer;
  private int bufferIndex = 0;
  private double valueSum = 0;
  
  public CircularBuffer(int bufferSize) {
    circularBuffer = new double[bufferSize];
  }
  
  public void inputValue(double value) {
    valueSum += value;
    valueSum -= circularBuffer[bufferIndex];
    circularBuffer[bufferIndex] = value;
    bufferIndex = (bufferIndex + 1) % circularBuffer.length;
  }
  
  public double getAverage() {
    return valueSum / circularBuffer.length;
  }
  
}