// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/ADXRS450_Gyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <rev/CANSparkMax.h>

#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() { m_gyro.Reset(); }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::second_t period);
  void UpdateOdometry();

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      std::numbers::pi};  // 1/2 rotation per second
  
  frc::ADXRS450_Gyro m_gyro;
  
 private:
  frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
  frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
  frc::Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
  frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

/*                         
const int driveMotorChannel,
const rev::CANSparkMax::MotorType driveMotorType,
const int turningMotorChannel,
const rev::CANSparkMax::MotorType turningMotorType,
const int turningEncoderChannel,
const std::string turningEncoderCanbus
*/

  SwerveModule m_frontLeft{0, rev::CANSparkMax::MotorType::kBrushless, 4, rev::CANSparkMax::MotorType::kBrushless, 1, "rio"};
  SwerveModule m_frontRight{1, rev::CANSparkMax::MotorType::kBrushless, 5, rev::CANSparkMax::MotorType::kBrushless, 2, "rio"};
  SwerveModule m_backLeft{2, rev::CANSparkMax::MotorType::kBrushless, 6, rev::CANSparkMax::MotorType::kBrushless, 3, "rio"};
  SwerveModule m_backRight{3, rev::CANSparkMax::MotorType::kBrushless, 8, rev::CANSparkMax::MotorType::kBrushless, 4, "rio"};



  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{
      m_kinematics,
      m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()}};
};
