package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHandler;

public class IntakeCoral extends Command {

    public IntakeCoral() {
        addRequirements(CoralHandler.getInstance());
    }

    public void execute() {
        CoralHandler.getInstance().intakeCoral();
    }

    public boolean isFinished() {
        return CoralHandler.getInstance().hasCoral();
    }
}
