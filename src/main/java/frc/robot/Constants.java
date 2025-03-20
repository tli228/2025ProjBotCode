// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(15.1);

  public static class CANConfig {
    public static final int END_EFFECTOR_MOTOR = 30;
    public static final int END_EFFECTOR_TILT = 31;
    public static final int END_EFFECTOR_INTAKE = 27; //(The funnel motor)
    public static final int ELEVATOR_LEFT = 25;
    public static final int ELEVATOR_RIGHT = 29;
    public static final int CLIMB_WINCH = 26;
    public static final int CLIMB_GRAB = 32;

    public static final int LIMIT_SWITCH_CHANNEL = 0;
  }

  public static class DrivetrainConfig {
    public static final double MAX_DRIVE_SPEED = 10.0; // m/s
    public static final double MAX_TURN_SPEED = 200.0; // deg/s
    public static final double SLOWMODE_FACTOR = 0.4;
    public static final PIDConstants DRIVE_PID = new PIDConstants(0.5, 0, 0);
    public static final PIDConstants TURN_PID = new PIDConstants(0.5, 0, 0);
  }
  public static class ElevatorConstants{
    public static final double L1Height = 20;
    public static final double L2Height = 173;
    public static final double L3Height = 302;
    public static final double L4Height = 507;
    public static final double A1Height = 104;
    public static final double A2Height = 165;
  }
  public static class tiltConstants{
    public static double startingHeight = 15.25;
    public static double endingHeight = 59.25;
    public static double startingEncoder = 0;
    public static double endingEncoder = 110;

    public static double inchesPerEncoder = (endingHeight - startingHeight)/(endingEncoder - startingEncoder);

    public static final double tiltOffset = -5.928567; //Please put this in degrees and change it according to what you measure.

    public static final double tiltMinSpeed = 1;
    public static final double tiltMaxSpeed = 2.5;

    public static final double tiltL1 = 3.380949;
    public static final double tiltL2 = 4.85710;
    public static final double tiltL3 = .928572;
    public static final double tiltL4 = 3.499997;
    public static final double tiltA1 = 22.118935;
    public static final double tiltA2 = 22.118935;
  }

  public static class SystemConfig {
    public static final PIDController PIVOT_PID = new PIDController(0.1, 0, 0);
    public static final PIDController SHOOTER_PID = new PIDController(0.1, 0, 0);
    public static final PIDController ELEVATOR_PID = new PIDController(0.1, 0, 0);
    public static final double PIVOT_TOLERANCE = 2; // deg
    public static final double SHOOTER_TOLERANCE = 2; // deg
    public static final double ELEVATOR_TOLERANCE = 0.1; // m

    public static final double ELEVATOR_SPEED = 0.7;
    public static final double PIVOT_SPEED = 0.5;
    public static final double SHOOTER_SPEED = 0.5;
    public static final double GRABBER_SPEED = 0.5;
    public static final double CORAL_INTAKE_SPEED = 0.5;
    public static final double FLYWHEEL_PROCESSOR_SPEED = 0.2;
    public static final double FLYWHEEL_NET_SPEED = 0.8;
    public static final double INDEXER_SPEED = 0.5;

    public static final double PIVOT_CONVERSION = 1;
    public static final double ELEVATOR_CONVERSION = 1;
    public static final double SHOOTER_CONVERSION = 1;

    // seconds
    public static final double SHOOTER_SPINUP_TIME = 2.0;
    public static final double SHOOTER_SHOOT_TIME = 2.0;
    public static final double SHOOTER_INTAKE_TIME = 0.5;
    public static final double CORAL_INTAKE_TIME = 1.0;
    public static final double CORAL_OUTTAKE_TIME = 1.0;
    public static final double GRABBER_TIME = 1.0;

    public static final double CLIMBER_DEFAULT_POSITION = 0;
    public static final double CLIMBER_CLIMB_POSITION = 0.5;
  }
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {


    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final int DRIVER_PORT         = 0;
    public static final int OPERATOR_PORT       = 1;
  }
}
