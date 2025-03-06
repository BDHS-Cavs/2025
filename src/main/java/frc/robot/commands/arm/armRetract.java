package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class armRetract extends Command{
    public armRetract(){
        addRequirements(RobotContainer.arm);
    }

    public void execute() {
        RobotContainer.arm.armRetract();
    }

    public void end(boolean interrupted) {
        RobotContainer.arm.armExtensionStop();
    }

}