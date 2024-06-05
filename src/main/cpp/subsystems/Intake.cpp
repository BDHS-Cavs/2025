// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "subsystems/Intake.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/raw_ostream.h> // for wpi outs()

Intake::Intake(){
    SetName("Intake");
    SetSubsystem("Intake");
    
	m_intakeMotor.SetInverted(false);
}

void Intake::Periodic() {
    // Put code here to be run every loop
}

void Intake::SimulationPeriodic() {
}

// Put methods for controlling this subsystem here and call from commands

void Intake::IntakeRun(){ 
    // Run Intake
        m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -0.9); 
} 

void Intake::IntakeExpel(){
    // Run Expel
    m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0.9);
}

void Intake::IntakeStop(){
    // stop the intake motor
    m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0.0);
}