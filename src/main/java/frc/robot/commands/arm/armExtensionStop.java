package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class armExtensionStop extends Command{
    public armExtensionStop(){
        addRequirements(RobotContainer.arm);
    }

    public void execute() {
        RobotContainer.arm.armExtensionStop();
    }

    public void end(boolean interrupted) {
    }

}