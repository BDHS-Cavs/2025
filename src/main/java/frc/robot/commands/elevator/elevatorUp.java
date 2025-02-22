package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class elevatorUp extends Command{
    public elevatorUp(){
        addRequirements(RobotContainer.elevator);
    }

    public void execute() {
        RobotContainer.elevator.elevatorUp();
    }

    public void end(boolean interrupted) {
        RobotContainer.elevator.elevatorStop();
    }

}