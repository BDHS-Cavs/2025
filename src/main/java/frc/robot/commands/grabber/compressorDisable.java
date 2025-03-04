package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber;

public class compressorDisable extends Command{
    grabber grabber;
    public compressorDisable(){
        addRequirements(grabber);
    }

    public void initialize() {
        //nothing
    }

    public void execute() {
        grabber.compressorDisable();
    }

    public void end(boolean interrupted) {
        //nothing
    }
}