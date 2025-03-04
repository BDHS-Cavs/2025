package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber;

public class wristRotate extends Command{
    grabber grabber;
    public wristRotate(){
        addRequirements(grabber);
    }

    public void initialize() {
        //nothing
    }

    public void execute() {
        grabber.wristRotate();
    }

    public void end(boolean interrupted) {
        //nothing
    }
}