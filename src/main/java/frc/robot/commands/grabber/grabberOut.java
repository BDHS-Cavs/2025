package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber;

public class grabberOut extends Command{
    grabber grabber;
    public grabberOut(){
        addRequirements(grabber);
    }

    public void execute() {
        grabber.grabberOut();
    }

    public void end(boolean interrupted) {
        grabber.grabberStop();
    }

}