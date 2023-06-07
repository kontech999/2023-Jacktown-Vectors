// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final int FRONT_LEFT_ID = 4;
  public static final int FRONT_RIGHT_ID = 1;
  public static final int BACK_LEFT_ID = 2;
  public static final int BACK_RIGHT_ID = 3;

  public static final int ARM_MOTOR_1 = 11;
  public static final int ARM_MOTOR_2 = 10;
  public static final int SLED_MOTOR_ID = 7;

  // Flip thses
  public static final int CLAW_SOLENOID_ID1 = 1;
  public static final int CLAW_SOLENOID_ID2 = 0;
  public static final int REVPH_ID = 5;

  public static final int ARM_ENCODER = 7;
  public static final int SLED_ENCODER_ID1 = 8;
  public static final int SLED_ENCODER_ID2 = 9;

  public static final int EXTEND_LIMITSWITCH_ID = 1;
  public static final int RETRACT_LIMITSWITCH_ID = 0;

  public static final int LEFT_JOYSTICK_ID = 0;
  public static final int RIGHT_JOYSTICK_ID = 1;
  public static final int XBOXCONTROLLER_ID = 2;

  public static final int LEFT_JOYSTICK_Y_AXIS_ID = 1;
  public static final int RIGHT_JOYSTICK_Y_AXIS_ID = 1;
  public static final int RIGHT_JOYSTICK_X_AXIS_ID = 0;
  public static final int LEFT_JOYSTICK_X_AXIS_ID = 0;

  public static final int BUTTON_12_ID = 12;
  public static final int BUTTON_10_ID = 10;
  public static final int BUTTON_9_ID = 9;
  public static final int BUTTON_8_ID = 8;
  public static final int BUTTON_7_ID = 7;
  public static final int BUTTON_2_ID = 2;
  public static final int BUTTON_1_ID = 1;

  public static final double LIMELIGHT_DEGREES_THRESHOLD = 0.5;
  public static final double LIMELIGHT_P = 0.03;

  public static final double DRIVE_AUTO_KP = 0.00002;
  public static final double DRIVE_AUTO_ROTATE_KP = 0.0042;
  public static final double SLED_MOTOR_DRIVE_KP = 0.0005;
  public static final double AB_MOTOR_DRIVE_KP = 7;

  public static final double SPIN_SPEED = 0.3;
  public static final double ABOFFSET = 0.03;
  public static double NAVX_OFFSET;
  public static final int kTimeoutsMs = 30;


  public static final double MAX_ARM_ENCODER = 0.88;
  public static final double MIN_ARM_ENCODER = 0.63;

  public static final double SLED_HOME = 300;
  public static final double SLED_LOADING = 300;
  public static final double SLED_LOW = 1680;
  public static final double SLED_MEDIUM = 5775;
  public static final double SLED_HIGH = 13000;
  // Sau
  /*
   * 17
   * 0.06
   * 1.5
   * 0.0046
   * 
   */
  // these are competition values from lakeview
  public static final double ANGLE_WHEN_ENABLES = 13.5;
  public static final double POWER_WHEN_PID_ENABLES = 0.055;
  // 3/28/23 Reed: changing scalar value from 0.28 to 1.1 to account for nathan's
  // changes at lakeview
  public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
  public static final double BEAM_BALANACED_DRIVE_KP = 0.023;
  public static final double BACK_NAVX_SCALAR_VALUE = 0.0055;
  public static final double FORWARD_NAVX_SCALAR_VALUE = 0.0070;
  public static final double CENTER_NAVX_SCALAR_VALUE = 0.0;

  /* 
  // This if for Field Home
  public static final double ANGLE_WHEN_ENABLES = 10;
  public static final double POWER_WHEN_PID_ENABLES = 0.055;
  // 3/28/23 Reed: changing scalar value from 0.28 to 1.1 to account for nathan's
  // changes at lakeview
  public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
  public static final double BEAM_BALANACED_DRIVE_KP = 0.023;
  public static final double BACK_NAVX_SCALAR_VALUE = 0.0055;
  public static final double FORWARD_NAVX_SCALAR_VALUE = 0.008;
  public static final double CENTER_NAVX_SCALAR_VALUE = 0.0;*/

}
