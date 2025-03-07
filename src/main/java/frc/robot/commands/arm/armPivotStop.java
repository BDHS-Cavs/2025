package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class armPivotStop extends Command{
    public armPivotStop(){
        addRequirements(RobotContainer.arm);
    }

    public void execute() {
        RobotContainer.arm.armPivotStop();
    }

    public void end(boolean interrupted) {
    }

}