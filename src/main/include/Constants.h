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
//driving ports

constexpr int kFrontLeftDriveMotorPort = 3;                //sm4d
constexpr int kBackLeftDriveMotorPort = 5;                 //sm3d
constexpr int kFrontRightDriveMotorPort = 9;               //sm1d
constexpr int kBackRightDriveMotorPort = 7;                //sm2d

constexpr int kFrontLeftTurningMotorPort = 4;              //sm4s
constexpr int kBackLeftTurningMotorPort = 6;               //sm3s
constexpr int kFrontRightTurningMotorPort = 10;            //sm1s
constexpr int kBackRightTurningMotorPort = 2;              //sm2s

constexpr int kFrontLeftTurningEncoderPort = 1;              //sm4s
constexpr int kBackLeftTurningEncoderPort = 2;               //sm3s
constexpr int kFrontRightTurningEncoderPort = 3;             //sm1s
constexpr int kBackRightTurningEncoderPort = 4;              //sm2s

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
//TODO constexpr int TIMEOUT = 0;                           //time? like 0ms?
//TODO constexpr double L = 20.125;                         //wheelbase
//TODO constexpr double W = 22.25;                          //trackwidth
//TODO constexpr double COUNTPERDEG = 0.0;                  //nicks number is 16.2539
//TODO constexpr double angleP = 2.0;                       //TODO find              //nicks numbers: double angleP = 1.03858, angleI = 0.004, angleD = 8, angleF = 0.51, angleV = 4012, angleA = 4012;
//TODO constexpr double angleI = 0.0;                       //TODO find
//TODO constexpr double angleD = 0.0;                       //TODO find
//TODO constexpr double angleF = 2.91;                      //(100% (the speed we ran it at) * 1023) / 263 (phoenix tuner x pid0 velocity while running at 100% speed)
//TODO constexpr int angleV = 132;                          //motion magic
//TODO constexpr int angleA = 132;                          //motion magic

//joystick stuff
//TODO constexpr double DEADZONE_XY = 0.1;
//TODO constexpr double DEADZONE_Z = 0.3;

    // How sensitive the Z axis is.
    // 0 = no effect
    // 1 = square the output
    // 2 = cube the output
//TODO constexpr int SENSITIVITY_Z = 2;

    // Reduces the maximum output for the Z axis.
    // 1 = full speed
    // 2 = half speed
    // etc
//TODO constexpr int REDUCER_Z = 2;


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
//TODO constexpr double kWheelDiameterMeters = 0.1016;
//TODO needed? constexpr double kDriveEncoderDistancePerPulse =
//TODO needed?     // Assumes the encoders are directly mounted on the wheel shafts
//TODO needed?     (kWheelDiameterMeters * std::numbers::pi) /
//TODO needed?     static_cast<double>(kDriveEncoderCPR);

//TODO needed? constexpr double kTurningEncoderDistancePerPulse =
//TODO needed?     // Assumes the encoders are directly mounted on the wheel shafts
//TODO needed?     (std::numbers::pi * 2) / static_cast<double>(kTurningEncoderCPR);

//TODO constexpr double kPModuleTurningController = 1;
//TODO constexpr double kPModuleDriveController = 1;
}  // namespace ModuleConstants

namespace AutoConstants {
//TODO constexpr auto kMaxSpeed = 3_mps;
//TODO constexpr auto kMaxAcceleration = 3_mps_sq;
//TODO constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
//TODO constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

//TODO constexpr double kPXController = 0.5;
//TODO constexpr double kPYController = 0.5;
//TODO constexpr double kPThetaController = 0.5;

//

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
//TODO constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants
