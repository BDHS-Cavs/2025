package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class wristRotate extends Command{
    public wristRotate(){
        addRequirements(RobotContainer.grabber);
    }

    public void execute() {
        RobotContainer.grabber.wristRotate();
    }

    public void end(boolean interrupted) {
    }

}