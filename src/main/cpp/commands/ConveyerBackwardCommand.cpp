// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "commands/ConveyerBackwardCommand.h"

ConveyerBackwardCommand::ConveyerBackwardCommand(Conveyer* m_conveyer)
:m_conveyer(m_conveyer){

    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("ConveyerBackwardCommand");
    AddRequirements(m_conveyer);
}

// Called just before this Command runs the first time
void ConveyerBackwardCommand::Initialize() {
   m_conveyer->ConveyerBackward();
}

// Called repeatedly when this Command is scheduled to run
void ConveyerBackwardCommand::Execute() {
}

// Make this return true when this Command no longer needs to run execute()
bool ConveyerBackwardCommand::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void ConveyerBackwardCommand::End(bool interrupted) {
    m_conveyer->ConveyerStop();
}

bool ConveyerBackwardCommand::RunsWhenDisabled() const {
    return false;
}