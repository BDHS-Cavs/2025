package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class compressorDisable extends Command{
    public compressorDisable(){
        addRequirements(RobotContainer.grabber);
    }

    public void execute() {
        RobotContainer.grabber.compressorDisable();;
    }

    public void end(boolean interrupted) {
    }

}