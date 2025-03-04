package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber;

public class compressorEnable extends Command{
    grabber grabber;
    public compressorEnable(){
        addRequirements(grabber);
    }

    public void execute() {
        grabber.compressorEnable();;
    }

    public void end(boolean interrupted) {
    }

}