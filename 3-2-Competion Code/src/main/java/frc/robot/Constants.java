// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

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

  public static final int ELEVATOR_MOTOR_ID = 10;

  public static final int AB_MOTOR_1 = 11;
  public static final int AB_MOTOR_2 = 10;
  public static final int BC_MOTOR_2 = 8;
  public static final int BC_MOTOR_1 = 9;

  public static final int CLAW_SOLENOID_ID1 = 4;
  public static final int CLAW_SOLENOID_ID2 = 5;
  public static final int REVPH_ID = 5;


  public static final int AB_ENCODER = 7;
  public static final int BC_ENCODER = 8;

  public static final int LEFT_JOYSTICK_ID = 0;
  public static final int RIGHT_JOYSTICK_ID = 1;
  public static final int XBOXCONTROLLER_ID = 2;

  public static final int LEFT_JOYSTICK_Y_AXIS_ID = 1;
  public static final int RIGHT_JOYSTICK_Y_AXIS_ID = 1;
  public static final int RIGHT_JOYSTICK_X_AXIS_ID = 0;
  public static final int LEFT_JOYSTICK_X_AXIS_ID = 0;

  public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
  public static final double BEAM_BALANACED_DRIVE_KP = 0.005;
  // public static final double BEAM_BALANACED_DRIVE_kD = 0.01;
  public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 0.75;
  public static final int BUTTON_12_ID = 12;
  public static final int BUTTON_10_ID = 10;
  public static final int BUTTON_9_ID = 9;
  public static final int BUTTON_8_ID = 8;
  public static final int BUTTON_7_ID = 7;
  public static final int BUTTON_2_ID = 2;
  public static final int BUTTON_1_ID = 1;

  public static final int ELEVATOR_ENCODER1 = 0;
  public static final int ELEVATOR_ENCODER2 = 1;
  public static final double DEGREES_THRESHOLD = 0.5;
  public static final int CLAW_MOTOR_ID = 7;

  public static final double LIMELIGHT_P = 0.03;
  public static final double DRIVE_AUTO_KP = 0.00002;
  public static final double DRIVE_AUTO_ROTATE_KP = 0.005;
  public static final double BC_MOTOR_DRIVE_KP = 5;
  public static final double AB_MOTOR_DRIVE_KP = 7;

  public static final double SPIN_SPEED = 0.3;
public static final double ABOFFSET = 0.02;

}
