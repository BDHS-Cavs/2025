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
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/ControlMode.h>

#include "Constants.h"

class Conveyer: public frc2::SubsystemBase {

private:
    ctre::phoenix::motorcontrol::can::TalonSRX m_conveyerMotor{ConveyerConstants::kConveyerMotorPort}; //tuned value
    
public:
    Conveyer();

    void Periodic() override;
    void SimulationPeriodic() override;
    void ConveyerForward();
    void ConveyerBackward();
    void ConveyerStop();
};