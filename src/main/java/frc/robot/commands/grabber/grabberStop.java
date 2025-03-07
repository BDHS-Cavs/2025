package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class grabberStop extends Command{
    public grabberStop(){
        addRequirements(RobotContainer.grabber);
    }

    public void execute() {
        RobotContainer.grabber.grabberStop();
    }

    public void end(boolean interrupted) {
    }

}