package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class elevatorDownBypass extends Command{
    public elevatorDownBypass(){
        addRequirements(RobotContainer.elevator);
    }

    public void execute() {
        RobotContainer.elevator.elevatorDownBypass();
    }

    public void end(boolean interrupted) {
        RobotContainer.elevator.elevatorStop();
    }

}