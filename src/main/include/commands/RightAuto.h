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

#include <frc2/command/CommandBase.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Climber.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include "subsystems/Conveyer.h"

class RightAuto: public frc2::CommandHelper<frc2::Command, RightAuto> {

public:

    explicit RightAuto(Drivetrain* m_swerve, Climber* m_climber, Shooter* m_shooter, Intake* m_intake, Conveyer* m_conveyer);

void Initialize() override;
void Execute() override;
bool IsFinished() override;
void End(bool interrupted) override;
bool RunsWhenDisabled() const override;


private:
    bool       m_firstTime;
    frc::Timer m_timer; 

    Drivetrain*     m_swerve;
    Climber*        m_climber;
    Shooter*        m_shooter;
    Intake*         m_intake;
    Conveyer*       m_conveyer;

//units::velocity::meters_per_second_t autoXSpeed = 0.2_mps;
//units::velocity::meters_per_second_t autoYSpeed = 0.2_mps;
//units::velocity::meters_per_second_t autoRot = 0.2_mps;
//bool autoFieldRelative = false;                          

};