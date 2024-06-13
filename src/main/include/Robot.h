// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <cameraserver/CameraServer.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  void DriveWithJoystick(bool fieldRelative);
  void TeleopPeriodic() override { DriveWithJoystick(true); }
  void AutonomousPeriodic() override;
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void TeleopInit() override;
  void TestPeriodic() override;


  void ConfigureButtonBindings();

  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  RobotContainer* m_container = RobotContainer::GetInstance();


 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.

  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand = nullptr;

  cs::UsbCamera usbcamera1;
  cs::UsbCamera usbcamera2;

  cs::VideoSink cameraserver;
};