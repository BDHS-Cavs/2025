// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain(){
    m_gyro.Reset();
    SetName("Drivetrain");
    SetSubsystem("Drivetrain");
}
void Drivetrain::Periodic() {
    // Put code here to be run every loop
}

void Drivetrain::SimulationPeriodic() {
}
void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative,
                       units::second_t period) {
  auto states =
      m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
          fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                              xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                        : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
          period));

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);

  //TODO brake or coast mode
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_backLeft.GetPosition(), m_backRight.GetPosition()});
}

void Drivetrain::DriveStop() {
    this->Drive(0_mps, 0_mps, 0_rad_per_s, false, 0_s);
}

void Drivetrain::AutoDriveBackwards() {
    this->Drive(-0.2_mps, -0.2_mps, 0_rad_per_s, false, 0_s);
}

void Drivetrain::AutoDriveForwards() {
    this->Drive(0.2_mps, 0.2_mps, 0_rad_per_s, false, 0_s);
}

void Drivetrain::AutoRotateLeft() {
    this->Drive(0.2_mps, 0.2_mps, 0.2_rad_per_s, false, 0_s);
}

void Drivetrain::AutoRotateRight() {
    this->Drive(0.2_mps, 0.2_mps, -0.2_rad_per_s, false, 0_s);
}