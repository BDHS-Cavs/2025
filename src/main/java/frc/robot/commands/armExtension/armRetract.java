package frc.robot.commands.armExtension;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class armRetract extends Command{
    public armRetract(){
        addRequirements(RobotContainer.armExtension);
    }

    public void execute() {
        RobotContainer.armExtension.armRetract();
    }

    public void end(boolean interrupted) {
        RobotContainer.armExtension.armExtensionStop();
    }

}