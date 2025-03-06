package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class armDown extends Command{
    public armDown(){
        addRequirements(RobotContainer.arm);
    }

    public void execute() {
        RobotContainer.arm.armDown();
    }

    public void end(boolean interrupted) {
        RobotContainer.arm.armPivotStop();
    }

}