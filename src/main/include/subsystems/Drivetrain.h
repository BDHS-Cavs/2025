// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc2/command/SubsystemBase.h>

#include <frc/ADXRS450_Gyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <rev/CANSparkMax.h>

#include "SwerveModule.h"

#include "Constants.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain: public frc2::SubsystemBase {
 public:
  Drivetrain();

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::second_t period);
  void Periodic() override;
  void SimulationPeriodic() override;
  void UpdateOdometry();

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      std::numbers::pi};  // 1/2 rotation per second - todo ??? half?
  
  frc::ADXRS450_Gyro m_gyro;
  
 private:
  frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};//TODO find
  frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
  frc::Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
  frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

/*                         
int driveMotorChannel
rev::CANSparkMax::MotorType driveMotorType
int turningMotorChannel
rev::CANSparkMax::MotorType turningMotorType
int turningEncoderChannel
std::string turningEncoderCanbus
*/

  SwerveModule m_frontLeft{DriveConstants::kFrontLeftDriveMotorPort, rev::CANSparkMax::MotorType::kBrushless, DriveConstants::kFrontLeftTurningMotorPort, rev::CANSparkMax::MotorType::kBrushless, DriveConstants::kFrontLeftTurningEncoderPort, "rio"};
  SwerveModule m_frontRight{DriveConstants::kFrontRightDriveMotorPort, rev::CANSparkMax::MotorType::kBrushless, DriveConstants::kFrontRightTurningMotorPort, rev::CANSparkMax::MotorType::kBrushless, DriveConstants::kFrontRightTurningEncoderPort, "rio"};
  SwerveModule m_backLeft{DriveConstants::kBackLeftDriveMotorPort, rev::CANSparkMax::MotorType::kBrushless, DriveConstants::kBackLeftTurningMotorPort, rev::CANSparkMax::MotorType::kBrushless, DriveConstants::kBackLeftTurningEncoderPort, "rio"};
  SwerveModule m_backRight{DriveConstants::kBackRightDriveMotorPort, rev::CANSparkMax::MotorType::kBrushless, DriveConstants::kBackRightTurningMotorPort, rev::CANSparkMax::MotorType::kBrushless, DriveConstants::kBackRightTurningEncoderPort, "rio"};



  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{
      m_kinematics,
      m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()}};
};
