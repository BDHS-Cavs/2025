// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

#include "RobotContainer.h"

#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
//#include "frc/shuffleboard/Shuffleboard.h" //todo check it out

RobotContainer* RobotContainer::m_robotContainer = NULL;

RobotContainer::RobotContainer() 
{
    frc::SmartDashboard::PutData(&m_swerve);
    //frc::SmartDashboard::PutData(&m_climber);
    //frc::SmartDashboard::PutData(&m_shooter);
    //frc::SmartDashboard::PutData(&m_intake);
    //frc::SmartDashboard::PutData(&m_conveyer);

  // Add commands to the autonomous command chooser
  //m_chooser.SetDefaultOption("Left Auto", &m_leftAuto);
  //m_chooser.AddOption("Right Auto", &m_rightAuto);
  //m_chooser.AddOption("Center Auto", &m_centerAuto);

  frc::SmartDashboard::PutData(&m_chooser);
	
  ConfigureButtonBindings();
}




RobotContainer* RobotContainer::GetInstance() {
    if (m_robotContainer == NULL) {
        m_robotContainer = new RobotContainer();
    }
    return(m_robotContainer);
}

void RobotContainer::ConfigureButtonBindings() {                                                           // MAKE SURE YOU ARE ON THE "X" NOT "D" ON BACK OF CONTROLLER
//frc2::JoystickButton m_controllerButton1{&m_controller, (int)frc::XboxController::Button::kA};             // Shooter Retract (1)   (A)
//frc2::JoystickButton m_controllerButton2{&m_controller, (int)frc::XboxController::Button::kB};             // Climber Raise (2)     (B)
//frc2::JoystickButton m_controllerButton3{&m_controller, (int)frc::XboxController::Button::kX};             // Climber Lower (3)     (X)
//frc2::JoystickButton m_controllerButton4{&m_controller, (int)frc::XboxController::Button::kY};             // Shooter Shoot (4)     (Y)
//frc2::JoystickButton m_controllerButton5{&m_controller, (int)frc::XboxController::Button::kLeftBumper};    // Intake Expel (5)      (LB)
//frc2::JoystickButton m_controllerButton6{&m_controller, (int)frc::XboxController::Button::kRightBumper};   // Intake Run (6)        (RB)
//frc2::POVButton m_controllerButton7{&m_controller, (int)270, 0};          // Conveyer Backward (7) (d pad left)
//frc2::POVButton m_controllerButton8{&m_controller, (int)90, 0};         // Conveyer Forward (8)  (d pad right)

//m_controllerButton1.WhileTrue(ShooterRetractCommand(&m_shooter).ToPtr());                                    // Shooter Retract (1)   (A)
//m_controllerButton2.WhileTrue(ClimberRaiseCommand(&m_climber).ToPtr());                                    // Climber Raise (2)     (B)
//m_controllerButton3.WhileTrue(ClimberLowerCommand(&m_climber).ToPtr());                                    // Climber Lower (3)     (X)
//m_controllerButton4.WhileTrue(ShooterShootCommand(&m_shooter).ToPtr());                                  // Shooter Shoot (4)     (Y)
//m_controllerButton5.WhileTrue(IntakeExpelCommand(&m_intake).ToPtr());                                      // Intake Expel (5)      (LB)
//m_controllerButton6.WhileTrue(IntakeRunCommand(&m_intake).ToPtr());                                        // Intake Run (6)        (RB)
//m_controllerButton7.WhileTrue(ConveyerBackwardCommand(&m_conveyer).ToPtr());                               // Conveyer Backward (pov left) (d pad left)
//m_controllerButton8.WhileTrue(ConveyerForwardCommand(&m_conveyer).ToPtr());                                // Conveyer Forward (pov right)  (d pad right)
}

/*frc::XboxController* RobotContainer::getJoystick() {
   return &m_drivecontroller;
}*/
//frc::PS4Controller* RobotContainer::getJoystick() {
frc::XboxController* RobotContainer::getJoystick() {
    return &m_drivecontroller;
}
frc::XboxController* RobotContainer::getController() {
   return &m_controller;
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // The selected command will be run in autonomous
  return m_chooser.GetSelected();
}