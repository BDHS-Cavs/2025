package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber;

public class grabberIn extends Command{
    grabber grabber;
    public grabberIn(){
        addRequirements(grabber);
    }

    public void execute() {
        grabber.grabberIn();
    }

    public void end(boolean interrupted) {
        grabber.grabberStop();
    }

}