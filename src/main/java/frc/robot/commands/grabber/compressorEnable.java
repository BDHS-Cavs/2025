package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class compressorEnable extends Command{
    public compressorEnable(){
        addRequirements(RobotContainer.grabber);
    }

    public void execute() {
        RobotContainer.grabber.compressorEnable();;
    }

    public void end(boolean interrupted) {
    }

}