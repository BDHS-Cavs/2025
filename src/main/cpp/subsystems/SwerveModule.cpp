// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

SwerveModule::SwerveModule(int driveMotorChannel,
                           rev::CANSparkMax::MotorType driveMotorType,
                           int turningMotorChannel,
                           rev::CANSparkMax::MotorType turningMotorType,
                           int turningEncoderChannel,
                           std::string turningEncoderCanbus)
    : m_driveMotor(driveMotorChannel, driveMotorType),
      m_turningMotor(turningMotorChannel, turningMotorType),
      m_turningEncoder(turningEncoderChannel, turningEncoderCanbus) {
        
        m_driveMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
        m_turningMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);//todo double check with jon

  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.

  //m_driveEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius / kEncoderResolution);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.

  //m_turningEncoder.SetDistancePerPulse(2 * std::numbers::pi / kEncoderResolution);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      -units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});

      m_driveMotor.RestoreFactoryDefaults();

     m_drivePIDController.SetP(DriveConstants::kdriveP);
     m_drivePIDController.SetI(DriveConstants::kdriveI);
     m_drivePIDController.SetD(DriveConstants::kdriveD);
     m_drivePIDController.SetIZone(DriveConstants::kdriveIZone);
     m_drivePIDController.SetFF(DriveConstants::kdriveFF);
     m_drivePIDController.SetOutputRange(DriveConstants::kdriveMinOutput, DriveConstants::kdriveMaxOutput);
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {
    units::turns_per_second_t{m_driveEncoder.GetVelocity() / 60} * (DriveConstants::kWheelDiameter * units::constants::pi / 1_tr), m_turningEncoder.GetAbsolutePosition().GetValue()
    };
}//TODO GetPosition or GetAbsolutePosition ??? - he said to use absolute https://www.reddit.com/r/FRC/comments/x30avg/

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {
    units::turn_t{m_driveEncoder.GetPosition()} * (DriveConstants::kWheelDiameter * units::constants::pi / 1_tr), m_turningEncoder.GetAbsolutePosition().GetValue()
  };
}//TODO GetPosition or GetAbsolutePosition ??? - he said to use absolute https://www.reddit.com/r/FRC/comments/x30avg/

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  frc::Rotation2d encoderRotation{
      m_turningEncoder.GetAbsolutePosition().GetValue()}; //TODO GetPosition or GetAbsolutePosition ??? - he said to use absolute https://www.reddit.com/r/FRC/comments/x30avg/

  // Optimize the reference state to avoid spinning further than 90 degrees
  auto state =
      frc::SwerveModuleState::Optimize(referenceState, encoderRotation);

  // Scale speed by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  state.speed *= (state.angle - encoderRotation).Cos();

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetVelocity(), state.speed.value());

  const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  const auto turnOutput = m_turningPIDController.Calculate(
      m_turningEncoder.GetAbsolutePosition().GetValue(), state.angle.Radians()); //TODO GetPosition or GetAbsolutePosition ??? - he said to use absolute https://www.reddit.com/r/FRC/comments/x30avg/

  const auto turnFeedforward = m_turnFeedforward.Calculate(
      m_turningPIDController.GetSetpoint().velocity);

  // Set the motor outputs.
  m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}
