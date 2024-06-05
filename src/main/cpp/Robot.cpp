// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
  //TODO frc::CameraServer::GetVideo(); //i dont know if we need this line because it says use this if you want to use vision processing on the roborio but we do it on the 2nd camera (the limelight 2+) with photonvision
  //TODO frc::CameraServer::StartAutomaticCapture(); //make usb cam (microsoft lifecam hd-3000) work

    usbcamera1 = frc::CameraServer::StartAutomaticCapture(0);
    usbcamera2 = frc::CameraServer::StartAutomaticCapture(1);
    //ethernetcamera = frc::CameraServer::AddAxisCamera("10.29.78.11");
//    ethernetcamera = frc::CameraServer::StartAutomaticCapture(2);

    usbcamera1.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
    usbcamera2.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
    //ethernetcamera.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);

    cameraserver = frc::CameraServer::GetServer();

  m_container->m_swerve.m_gyro.Calibrate(); //calibrate gyro
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  //gyro
    frc::SmartDashboard::PutNumber("gyro get angle", m_container->m_swerve.m_gyro.GetAngle());
    frc::SmartDashboard::PutNumber("gyro get rate", m_container->m_swerve.m_gyro.GetRate());

  //shooter
    frc::SmartDashboard::PutNumber("Shooter Encoder 1 Position", m_container->m_shooter.m_shooterEncoder1.GetPosition());
    frc::SmartDashboard::PutNumber("Shooter Encoder 2 Position", m_container->m_shooter.m_shooterEncoder2.GetPosition());

  //climber
    frc::SmartDashboard::PutNumber("Climber Encoder 1 Position", m_container->m_climber.m_climberEncoder1.GetPosition());
    frc::SmartDashboard::PutNumber("Climber Encoder 2 Position", m_container->m_climber.m_climberEncoder2.GetPosition());
  }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container->GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
//TODO CRASHES AUTO? but i dont think we need bc joystick disabled in auto anyway //  DriveWithJoystick(false);
//TODO remove? remove the whole function? m_container->m_swerve.UpdateOdometry();
//m_autonomousCommand->Execute();
}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.

  //TODO: might want to swerve update odometry and might want to reset climber/shooter encoders
  
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

/**
 * This function is called periodically during operator control.
 */
/*TODO remove? void Robot::TeleopPeriodic() {
  //DriveWithJoystick(true);

//swerve drive with joystick
//m_container->m_swerve.calculateVectors(m_container->getX(), m_container->getY(), m_container->getZ());
//m_container->m_swerve.calculateVectors(1, 1, 0);

}*/

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

void Robot::DriveWithJoystick(bool fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_container->m_controller.GetLeftY(), 0.02)) *
                        Drivetrain::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -m_yspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_container->m_controller.GetLeftX(), 0.02)) *
                        Drivetrain::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -m_rotLimiter.Calculate(
                         frc::ApplyDeadband(m_container->m_controller.GetRightX(), 0.02)) *
                     Drivetrain::kMaxAngularSpeed;

    m_container->m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, GetPeriod());
  }

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif