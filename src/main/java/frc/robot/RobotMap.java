/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;

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
    public static final int FRONT_LEFT_TALON_D = 21;
    public static final int FRONT_RIGHT_TALON_D = 23;
    public static final int BACK_LEFT_TALON_D = 25;
    public static final int BACK_RIGHT_TALON_D = 27;

    public static final int FRONT_LEFT_TALON_T = 22;
    public static final int FRONT_RIGHT_TALON_T = 24;
    public static final int BACK_LEFT_TALON_T = 26;
    public static final int BACK_RIGHT_TALON_T = 28;

    public static final int FRONT_LEFT_ABS_ENCODER_ID = 0;
    public static final int FRONT_RIGHT_ABS_ENCODER_ID = 1;
    public static final int BACK_LEFT_ABS_ENCODER_ID = 2;
    public static final int BACK_RIGHT_ABS_ENCODER_ID = 3;

    // public static final int kTurnP = 0;
    // public static final int kTurnI = 0;
    // public static final int kTurnD = 0;
    

    // public static final int FRONT_LEFT_TALON_E = 9;
    // public static final int FRONT_RIGHT_TALON_E = 10;
    // public static final int BACK_LEFT_TALON_E = 11;
    // public static final int BACK_RIGHT_TALON_E = 12;

  }

  public static final class Intake {
    public static final int INTAKE_VICTOR = 30; //change this to a victor
    // public static final int LEFT_INTAKE_PISTON_FORWARD = 0;
    // public static final int LEFT_INTAKE_PISTON_REVERSE = 1;

    // public static final int RIGHT_INTAKE_PISTON_FORWARD = 0;
    // public static final int RIGHT_INTAKE_PISTON_REVERSE = 1;

    public static final int PISTON_FORWARD = 0;
    public static final int PISTON_REVERSE = 3;
  }

  public static final class Shooter {
    public static final int SHOOTER_SPARK_MAX = 42;
    public static final int FEEDER_SPARK_MAX = 41;
  }

  public static final class LEDStrip {
    public static final int LED = 9;
  }

  public static final class PCH {
    public static final int PCH_CAN = 1;
    public static final int COMPRESSOR = 1;
    public static final int PRESSURE_SENSOR_CHANNEL = 0;
  }

  public static final class Aligner {
    public static final I2C.Port I2C_PORT = I2C.Port.kOnboard;
  }

  public static final class Climb {
    public static final int LEFT_WINCH_TALON = 25;
    public static final int RIGHT_WINCH_TALON = 26;

    public static final int PISTON_FORWARD = 1;
    public static final int PISTON_REVERSE = 2;

    public static final int LEFT_SWITCH_PISTON_FORWARD = 11;
    public static final int LEFT_SWITCH_PISTON_REVERSE = 11;

  }

  public static final class Controller {
    public static final int RAW_AXIS_X = 0;
    public static final int RAW_AXIS_Y = 1;
    
    public static final int RESET_GYRO_HEADING_BUTTON_ID = 5;
  }
}
