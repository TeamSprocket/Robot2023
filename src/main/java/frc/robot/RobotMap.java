/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// import edu.wpi.first.wpilibj.I2C;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public final class RobotMap {
  /*
   * 0 - PDP
   * 1 - PCM
   * 10..19 - Talon SRX
   * 20..29 - Talon FX
   * 30..39 - Victor SPX
   * 40..49 - Spark Max
   */
  public static final class Drivetrain {

    public static final int FRONT_LEFT_MOVE = 20;
    public static final int FRONT_LEFT_TURN = 21;

    public static final int FRONT_RIGHT_MOVE = 22;
    public static final int FRONT_RIGHT_TURN = 23;

    public static final int BACK_LEFT_MOVE = 24;
    public static final int BACK_LEFT_TURN = 25;
    
    public static final int BACK_RIGHT_MOVE = 26;
    public static final int BACK_RIGHT_TURN = 27;

    public static final int kTurnP = 0;
    public static final int kTurnI = 0;
    public static final int kTurnD = 0;
  }

  public static final class Intake {

    public static final int INTAKE = 00; //change this to a victor
    

    public static final int PISTON_FORWARD = 0;
    public static final int PISTON_REVERSE = 1;
    
  }

  public static final class LEDStrip {
    public static final int LED = 9;
  }

  public static final class PCH {
    public static final int PCH_CAN = 1;
    public static final int COMPRESSOR = 1;
    public static final int PRESSURE_SENSOR_CHANNEL = 0;
  }


}
