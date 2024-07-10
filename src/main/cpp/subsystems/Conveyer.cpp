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

#include "subsystems/Conveyer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/raw_ostream.h>


Conveyer::Conveyer(){
    SetName("Conveyer");
    SetSubsystem("Conveyer");

    m_conveyerMotor.SetInverted(false);
}

void Conveyer::Periodic() {
    // Put code here to be run every loop
}

void Conveyer::SimulationPeriodic() {
}

// Put methods for controlling this subsystem here and call from commands

void Conveyer::ConveyerForward(){ 
    // Run Conveyer Forward
    m_conveyerMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.2);
} 

void Conveyer::ConveyerBackward(){
    // Run Conveyer Backward
    m_conveyerMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.2);
}

void Conveyer::ConveyerStop(){
    // Stop the Conveyer motor
    m_conveyerMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
}