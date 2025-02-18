package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class grabberIn extends Command{
    public grabberIn(){
        addRequirements(RobotContainer.grabber);
    }

    public void execute() {
        RobotContainer.grabber.grabberIn();;
    }

    public void end(boolean interrupted) {
    }

}