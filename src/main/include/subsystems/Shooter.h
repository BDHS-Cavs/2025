// THIS FILE IS FROM 2024 - OUR 2025 CODE WILL PROBABLY NOT KEEP THIS FILE, IT IS JUST TO MAKE PROGRAMMING 2025 EASIER

// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class Shooter: public frc2::SubsystemBase {

private:
    rev::CANSparkMax m_shooterMotor1{ShooterConstants::kShooterMotor1Port, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_shooterMotor2{ShooterConstants::kShooterMotor2Port, rev::CANSparkMax::MotorType::kBrushless};
    //frc::MotorControllerGroup m_climberMotors{m_shooterMotor1, m_shooterMotor2};//todo delete? or use spark groups

public:
    Shooter();

    void Periodic() override;
    void SimulationPeriodic() override;
    void ShooterShoot();
    void ShooterRetract();
    void ShooterStop();



    rev::SparkRelativeEncoder m_shooterEncoder1 = m_shooterMotor1.GetEncoder();
    rev::SparkRelativeEncoder m_shooterEncoder2 = m_shooterMotor2.GetEncoder();

};