// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
//driving ports (TalonSRX)

/*
FOR THE MXP ENCODER PORTS
0 is 10
1 is 11
2 is 12
3 is 13
4 is 18
5 is 19
6 is 20
7 is 21
8 is 22
9 is 23 etc.
*/

constexpr int kFrontLeftMotorPort = 3;                //sm4d
constexpr int kRearLeftMotorPort = 5;                 //sm3d
constexpr int kFrontRightMotorPort = 9;               //sm1d
constexpr int kRearRightMotorPort = 7;                //sm2d

constexpr int kFrontLeftTurningMotorPort = 4;              //sm4s
constexpr int kRearLeftTurningMotorPort = 6;               //sm3s
constexpr int kFrontRightTurningMotorPort = 10;            //sm1s
constexpr int kRearRightTurningMotorPort = 2;              //sm2s

//constexpr int kFrontLeftTurningEncoderChannelA = 2;        //sm4s
//constexpr int kRearLeftTurningEncoderChannelA = 6;         //sm3s
//constexpr int kFrontRightTurningEncoderChannelA = 12;      //sm1s
//constexpr int kRearRightTurningEncoderChannelA = 18;       //sm2s

//constexpr int kFrontLeftTurningEncoderChannelB = 3;        //sm4s
//constexpr int kRearLeftTurningEncoderChannelB = 7;         //sm3s
//constexpr int kFrontRightTurningEncoderChannelB = 13;      //sm1s
//constexpr int kRearRightTurningEncoderChannelB = 19;       //sm2s

//constexpr bool kFrontLeftTurningEncoderReversed = false;   //sm4s
//constexpr bool kRearLeftTurningEncoderReversed = false;     //sm3s
//constexpr bool kFrontRightTurningEncoderReversed = false;  //sm1s
//constexpr bool kRearRightTurningEncoderReversed = false;    //sm2s

//constexpr int kFrontLeftDriveEncoderChannelA = 1;          //sm4d
//constexpr int kRearLeftDriveEncoderChannelA = 5;           //sm3d
//constexpr int kFrontRightDriveEncoderChannelA = 10;        //sm1d
//constexpr int kRearRightDriveEncoderChannelA = 8;          //sm2d

//constexpr int kFrontLeftDriveEncoderChannelB = 0;          //sm4d
//constexpr int kRearLeftDriveEncoderChannelB = 4;           //sm3d
//constexpr int kFrontRightDriveEncoderChannelB = 11;        //sm1d
//constexpr int kRearRightDriveEncoderChannelB = 9;          //sm2d

//constexpr bool kFrontLeftDriveEncoderReversed = false;     //sm4d
//constexpr bool kRearLeftDriveEncoderReversed = false;       //sm3d
//constexpr bool kFrontRightDriveEncoderReversed = false;    //sm1d
//constexpr bool kRearRightDriveEncoderReversed = false;      //sm2d

// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The SysId tool provides a convenient
// method for obtaining these values for your robot.
//TODO needed? constexpr auto ks = 1_V; //volts
//TODO needed? constexpr auto kv = 0.8 * 1_V * 1_s / 1_m; //volts * seconds / meters
//TODO needed? constexpr auto ka = 0.15 * 1_V * 1_s * 1_s / 1_m; //volts * seconds^2 / meters      //TODO KV AND KA ARE FOR MOTION MAGIC (not needed?)

// Example value only - as above, this must be tuned for your drive!
//TODO needed? constexpr double kPFrontLeftVel = 0.5;
//TODO needed? constexpr double kPRearLeftVel = 0.5;
//TODO needed? constexpr double kPFrontRightVel = 0.5;
//TODO needed? constexpr double kPRearRightVel = 0.5;


//pid stuff
constexpr int TIMEOUT = 0;                           //time? like 0ms?
constexpr double L = 20.125;                         //wheelbase
constexpr double W = 22.25;                          //trackwidth
constexpr double COUNTPERDEG = 0.0;                  //nicks number is 16.2539
constexpr double angleP = 2.0;                       //TODO find              //nicks numbers: double angleP = 1.03858, angleI = 0.004, angleD = 8, angleF = 0.51, angleV = 4012, angleA = 4012;
constexpr double angleI = 0.0;                       //TODO find
constexpr double angleD = 0.0;                       //TODO find
constexpr double angleF = 2.91;                      //(100% (the speed we ran it at) * 1023) / 263 (phoenix tuner x pid0 velocity while running at 100% speed)
constexpr int angleV = 132;                          //motion magic
constexpr int angleA = 132;                          //motion magic

//joystick stuff
constexpr double DEADZONE_XY = 0.1;
constexpr double DEADZONE_Z = 0.3;

    // How sensitive the Z axis is.
    // 0 = no effect
    // 1 = square the output
    // 2 = cube the output
constexpr int SENSITIVITY_Z = 2;

    // Reduces the maximum output for the Z axis.
    // 1 = full speed
    // 2 = half speed
    // etc
constexpr int REDUCER_Z = 2;


}  // namespace DriveConstants

namespace ClimberConstants {
//climber (SparkMax)
constexpr int kClimberMotor1Port = 14; //spark
constexpr int kClimberMotor2Port = 15; //spark
}

namespace IntakeConstants {
//intake (TalonSRX)
constexpr int kIntakeMotorPort = 1;
}

namespace ShooterConstants {
//shooter (SparkMax)
constexpr int kShooterMotor1Port = 12; //spark
constexpr int kShooterMotor2Port = 13; //spark
}

namespace ConveyerConstants {
//conveyer (TalonSRX)
constexpr int kConveyerMotorPort = 8;
}

namespace ModuleConstants {
//TODO needed? constexpr int kDriveEncoderCPR = 5;
//TODO needed? constexpr double kTurningEncoderCPR = 615;
constexpr double kWheelDiameterMeters = 0.1016;
//TODO needed? constexpr double kDriveEncoderDistancePerPulse =
//TODO needed?     // Assumes the encoders are directly mounted on the wheel shafts
//TODO needed?     (kWheelDiameterMeters * std::numbers::pi) /
//TODO needed?     static_cast<double>(kDriveEncoderCPR);

//TODO needed? constexpr double kTurningEncoderDistancePerPulse =
//TODO needed?     // Assumes the encoders are directly mounted on the wheel shafts
//TODO needed?     (std::numbers::pi * 2) / static_cast<double>(kTurningEncoderCPR);

constexpr double kPModuleTurningController = 1;
constexpr double kPModuleDriveController = 1;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

//

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants
