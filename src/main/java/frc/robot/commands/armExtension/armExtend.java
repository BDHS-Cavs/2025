package frc.robot.commands.armExtension;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class armExtend extends Command{
    public armExtend(){
        addRequirements(RobotContainer.armExtension);
    }

    public void execute() {
        RobotContainer.armExtension.armExtend();
    }

    public void end(boolean interrupted) {
        RobotContainer.armExtension.armExtensionStop();
    }

}