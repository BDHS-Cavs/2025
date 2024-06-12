// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

//#include <frc/Encoder.h>
#include <rev/SparkRelativeEncoder.h>
#include <ctre/phoenix6/CANcoder.hpp> //#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
//#include <frc/motorcontrol/PWMSparkMax.h>
#include <rev/CANSparkMax.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "Constants.h"

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, rev::CANSparkMax::MotorType driveMotorType, int turningMotorChannel, rev::CANSparkMax::MotorType turningMotorType,
               int turningEncoderChannel, std::string turningEncoderCanbus);
  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition();
  void SetDesiredState(const frc::SwerveModuleState& state);

 private:
  //TODO delete? static constexpr double kWheelRadius = 0.0508;
  //TODO delete? static constexpr int kEncoderResolution = 4096;

  static constexpr auto kModuleMaxAngularVelocity =
      std::numbers::pi * 1_rad_per_s;  // radians per second
  static constexpr auto kModuleMaxAngularAcceleration =
      std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2

  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  rev::SparkRelativeEncoder m_driveEncoder = m_driveMotor.GetEncoder();
  ctre::phoenix6::hardware::CANcoder m_turningEncoder;

  frc::PIDController m_drivePIDController{DriveConstants::kdriveP, DriveConstants::kdriveI, DriveConstants::kdriveD};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      DriveConstants::kturningP,
      DriveConstants::kturningI,
      DriveConstants::kturningD,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V,
                                                                3_V / 1_mps};
  frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
      1_V, 0.5_V / 1_rad_per_s};
};
