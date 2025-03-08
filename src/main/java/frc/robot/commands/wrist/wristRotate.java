package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class wristRotate extends Command{
    public wristRotate(){
        addRequirements(RobotContainer.wrist);
    }

    public void execute() {
        RobotContainer.wrist.wristRotate();
    }

    public void end(boolean interrupted) {
    }

}