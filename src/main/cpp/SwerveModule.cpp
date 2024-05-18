// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

/*  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               int turningEncoderChannel, std::string turningEncoderCanbus);*/

SwerveModule::SwerveModule(int driveMotorChannel,
                           rev::CANSparkMax::MotorType driveMotorType,
                           int turningMotorChannel,
                           rev::CANSparkMax::MotorType turningMotorType,
                           int turningEncoderChannel,
                           std::string turningEncoderCanbus)
    : m_driveMotor(driveMotorChannel, driveMotorType),
      m_turningMotor(turningMotorChannel, turningMotorType),
      m_turningEncoder(turningEncoderChannel, turningEncoderCanbus) {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  //TODO DELETE m_driveEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius /
  //TODO DELETE                                    kEncoderResolution);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.
  //TODO DELETE m_turningEncoder.SetDistancePerPulse(2 * std::numbers::pi /
  //TODO DELETE                                      kEncoderResolution);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      -units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi}); //TODO change to this, also change last 2 lines then. https://codedocs.revrobotics.com/cpp/classrev_1_1_spark_max_p_i_d_controller
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          units::radian_t{m_turningEncoder.GetPosition().GetValue()}}; //TODO GetPosition or GetAbsolutePosition ???
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {units::meter_t{m_driveEncoder.GetPosition()},
          units::radian_t{m_turningEncoder.GetPosition().GetValue()}}; //TODO GetPosition or GetAbsolutePosition ???
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  frc::Rotation2d encoderRotation{
      units::radian_t{m_turningEncoder.GetPosition().GetValue()}}; //TODO GetPosition or GetAbsolutePosition ???

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
      units::radian_t{m_turningEncoder.GetPosition().GetValue()}, state.angle.Radians()); //TODO GetPosition or GetAbsolutePosition ???

  const auto turnFeedforward = m_turnFeedforward.Calculate(
      m_turningPIDController.GetSetpoint().velocity);

  // Set the motor outputs.
  m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}
