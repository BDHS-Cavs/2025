// THIS FILE IS FROM 2024 - OUR 2025 CODE WILL PROBABLY NOT KEEP THIS FILE, IT IS JUST TO MAKE PROGRAMMING 2025 EASIER

// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "commands/ClimberRaiseCommand.h"

ClimberRaiseCommand::ClimberRaiseCommand(Climber* m_climber)
:m_climber(m_climber){

    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("ClimberRaiseCommand");
    AddRequirements(m_climber);
}

// Called just before this Command runs the first time
void ClimberRaiseCommand::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ClimberRaiseCommand::Execute() {
        m_climber->ClimberRaise();
}

// Make this return true when this Command no longer needs to run execute()
bool ClimberRaiseCommand::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void ClimberRaiseCommand::End(bool interrupted) {
    m_climber->ClimberStop();
}

bool ClimberRaiseCommand::RunsWhenDisabled() const {
    return false;
}