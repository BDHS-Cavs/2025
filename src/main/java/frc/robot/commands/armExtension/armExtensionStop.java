package frc.robot.commands.armExtension;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class armExtensionStop extends Command{
    public armExtensionStop(){
        addRequirements(RobotContainer.armExtension);
    }

    public void execute() {
        RobotContainer.armExtension.armExtensionStop();
    }

    public void end(boolean interrupted) {
    }

}