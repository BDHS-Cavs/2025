package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class wristRotateOther extends Command{
    public wristRotateOther(){
        addRequirements(RobotContainer.wrist);
    }

    public void execute() {
        RobotContainer.wrist.wristRotateOther();
    }

    public void end(boolean interrupted) {
    }

}