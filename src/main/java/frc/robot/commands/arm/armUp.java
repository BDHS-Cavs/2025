package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm;

public class armUp extends Command{
    arm arm;
    public armUp(){
        addRequirements(arm);
    }

    public void initialize() {
        //nothing
    }

    public void execute() {
        arm.armUp();
    }

    public void end(boolean interrupted) {
        arm.armStop();
    }
}