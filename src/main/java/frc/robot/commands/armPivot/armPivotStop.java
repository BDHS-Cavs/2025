package frc.robot.commands.armPivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class armPivotStop extends Command{
    public armPivotStop(){
        addRequirements(RobotContainer.armPivot);
    }

    public void execute() {
        RobotContainer.armPivot.armPivotStop();
    }

    public void end(boolean interrupted) {
    }

}