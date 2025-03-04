package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator;

public class elevatorDown extends Command{
    elevator elevator;
    public elevatorDown(){
        addRequirements(elevator);
    }

    public void initialize() {
        //nothing
    }

    public void execute() {
        elevator.elevatorDown();
    }

    public void end(boolean interrupted) {
        elevator.elevatorStop();
    }
}