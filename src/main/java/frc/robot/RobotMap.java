/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;

public final class RobotMap {
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
  }

  public static final class Controller {
    public static final int DRIVER = 0;
    public static final int OPERATOR = 1;
    
    public static final int RESET_GYRO_HEADING_BUTTON_ID = 1;
  }
}










// -=All=-
// DEPORT = RIGHT JOYSTICK
// HOME = X
// RESET = Funny Button

// -=CONES=-
// HIGH = Y
// MID = A
// LOW = LEFT JOYSTICK
// FLOOR = RIGHT BUMPER

// -=CUBES=-
// HIGH = B
// MID = Chair
// LOW = LEFT BUMPER







