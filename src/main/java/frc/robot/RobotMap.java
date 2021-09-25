/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static final int XBOX_PORT = 0;
  public static final int SWITCHBOX_PORT = 1;

  public static final int FRONT_LEFT_PIVOT = 6;
  public static final int FRONT_RIGHT_PIVOT = 5;
  public static final int REAR_LEFT_PIVOT = 7;
  public static final int REAR_RIGHT_PIVOT = 4;

  public static final int FRONT_LEFT_DRIVE = 2;
  public static final int FRONT_RIGHT_DRIVE = 1;
  public static final int REAR_LEFT_DRIVE = 3;
  public static final int REAR_RIGHT_DRIVE = 20;

  public static final int FRONT_LEFT_DIGITAL_INPUT = 1;
  public static final int FRONT_RIGHT_DIGITAL_INPUT = 2;
  public static final int REAR_LEFT_DIGITAL_INPUT = 0;
  public static final int REAR_RIGHT_DIGITAL_INPUT = 3;

  public static final int INTAKE = 15;
  public static final int INTAKE_SOLENOID_1 = 3;
  public static final int INTAKE_SOLENOID_2 = 4;

  public static final int TELESCOPE = 14;
  public static final int PTO = 1;

  public static final int CP_SOLONOID = 0;
  public static final int MANIPULATOR = 11;

  public static final int SHOOTER1 = 12;
  public static final int SHOOTER2 = 13;
  public static final int HOOD = 10;

  public static final int INDEX_MOTOR = 9;
  public static final int LIMIT_PORT_1 = 9;
  public static final int LIMIT_PORT_2 = 8;
  public static final int LIMIT_PORT_3 = 7; 
  public static final int LIMIT_PORT_5 = 6; 
  public static final int LIMIT_PORT_DECREMENT = 5;

  public static final int GUIDE = 2;
  public static final int CAMERA_SERVO = 9;
}