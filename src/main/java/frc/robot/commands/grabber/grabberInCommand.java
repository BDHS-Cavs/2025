package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class grabberInCommand extends Command{
    public grabberInCommand(){
        addRequirements(RobotContainer.grabber);
    }

    public void execute() {
        RobotContainer.grabber.grabberIn();
    }

    public void end(boolean interrupted) {
        RobotContainer.grabber.grabberStop();
    }

}