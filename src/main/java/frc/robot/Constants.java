// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
      // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class GrabberConstants
  {
    public static final int grabberMotorID = 10; //10 can
    public static final MotorType grabberMotorType = MotorType.kBrushless;

    public static final int wristSolenoidForwardID = 1; //pneumatic hub channel 1 forward
    public static final int wristSolenoidBackwardID = 2; //pneumatic hub channel 2 backward
  }

  public static class ArmConstants
  {
    public static final int armExtensionMotorID = 13; //13 can
    public static final int armPivotMotorID = 12; //12 can
    public static final MotorType armMotorType = MotorType.kBrushed;
    public static final int armExtensionLimitID = 5; //1 navx aio (4)
    public static final int armRetractionLimitID = 4; //0 navx aio (5)
  }

  public static class PneumaticConstants
  {
    public static final int pneumaticHubID = 11; //11 can
    public static final PneumaticsModuleType pneumaticHubModuleType = PneumaticsModuleType.REVPH;
    
    public static final double compressorMin = 80.0;
    public static final double compressorMax = 115.0;
  }

  public static class ElevatorConstants
  {
    public static final int elevatorMotorID = 9; //9 can
    public static final MotorType elevatorMotorType = MotorType.kBrushless;

    public static final int latchSolenoidID = 0; //channel 0, single channel solenoid
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
