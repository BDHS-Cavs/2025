package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm;

public class armDown extends Command{
    arm arm;
    public armDown(){
        addRequirements(arm);
    }

    public void initialize() {
        //nothing
    }

    public void execute() {
        arm.armDown();
    }

    public void end(boolean interrupted) {
        arm.armStop();
    }
}