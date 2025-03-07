package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class elevatorStop extends Command{
    public elevatorStop(){
        addRequirements(RobotContainer.elevator);
    }

    public void execute() {
        RobotContainer.elevator.elevatorStop();
    }

    public void end(boolean interrupted) {
    }

}