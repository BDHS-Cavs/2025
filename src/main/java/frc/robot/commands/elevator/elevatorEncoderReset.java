package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class elevatorEncoderReset extends Command{
    public elevatorEncoderReset(){
        addRequirements(RobotContainer.elevator);
    }

    public void execute() {
        RobotContainer.elevator.elevatorEncoderReset();
    }

    public void end(boolean interrupted) {
    }

}