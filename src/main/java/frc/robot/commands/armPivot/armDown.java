package frc.robot.commands.armPivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class armDown extends Command{
    public armDown(){
        addRequirements(RobotContainer.armPivot);
    }

    public void execute() {
        RobotContainer.armPivot.armDown();
    }

    public void end(boolean interrupted) {
        RobotContainer.armPivot.armPivotStop();
    }

}