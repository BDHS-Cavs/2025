package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class armUp extends Command{
    public armUp(){
        addRequirements(RobotContainer.arm);
    }

    public void execute() {
        RobotContainer.arm.armUp();;
    }

    public void end(boolean interrupted) {
    }

}