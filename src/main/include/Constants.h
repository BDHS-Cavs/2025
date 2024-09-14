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
constexpr int kFrontLeftDriveMotorPort = 8;                //sm4d
constexpr int kFrontLeftTurningMotorPort = 7;              //sm4s
constexpr int kFrontLeftTurningEncoderPort = 1;            //sm4se

constexpr int kBackLeftDriveMotorPort = 1;                 //sm3d
constexpr int kBackLeftTurningMotorPort = 2;               //sm3s
constexpr int kBackLeftTurningEncoderPort = 2;             //sm3se

constexpr int kFrontRightDriveMotorPort = 6;               //sm1d
constexpr int kFrontRightTurningMotorPort = 5;            //sm1s
constexpr int kFrontRightTurningEncoderPort = 3;           //sm1se

constexpr int kBackRightDriveMotorPort = 4;                //sm2d
constexpr int kBackRightTurningMotorPort = 3;              //sm2s
constexpr int kBackRightTurningEncoderPort = 4;            //sm2se

//driving pid
//constexpr double kdriveP = 1.0; //TODO try 5.0
//constexpr double kdriveI = 0.0;
//constexpr double kdriveD = 0.0;

//constexpr double kturningP = 1.0; //TODO try 5.0
//constexpr double kturningI = 0.0;
//constexpr double kturningD = 0.0;

constexpr double kdriveP = 6e-5;
constexpr double kdriveI = 1e-6;
constexpr double kdriveD = 0.0;
constexpr double kdriveIZone = 0.0;
constexpr double kdriveFF = 0.000015;
constexpr double kdriveMaxOutput = 1.0;
constexpr double kdriveMinOutput = -1.0;
constexpr double kturningP = 0.0;
constexpr double kturningI = 0.0;
constexpr double kturningD = 0.0;

constexpr units::inch_t kWheelDiameter = 4_in;

constexpr units::meter_t kWheelLocations = 0.17145_m;

}

namespace ClimberConstants {
//climber (SparkMax)
constexpr int kClimberMotor1Port = 14; //spark
constexpr int kClimberMotor2Port = 15; //spark
}

namespace IntakeConstants {
//intake (TalonSRX)
constexpr int kIntakeMotorPort = 1; //talonsrx
}

namespace ShooterConstants {
//shooter (SparkMax)
constexpr int kShooterMotor1Port = 12; //spark
constexpr int kShooterMotor2Port = 13; //spark
}

namespace ConveyerConstants {
//conveyer (TalonSRX)
constexpr int kConveyerMotorPort = 8; //talonsrx
}

namespace AutoConstants {
//auto constants
extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints; //TODO wtf is this

}

namespace OIConstants {
constexpr int kDriveControllerPort = 0;
constexpr int kControllerPort = 1;
}
