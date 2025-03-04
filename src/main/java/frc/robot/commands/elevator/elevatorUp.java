package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator;

public class elevatorUp extends Command{
    elevator elevator;
    public elevatorUp(){
        addRequirements(elevator);
    }

    public void initialize() {
        //nothing
    }

    public void execute() {
        elevator.elevatorUp();
    }

    public void end(boolean interrupted) {
        elevator.elevatorStop();
    }
}