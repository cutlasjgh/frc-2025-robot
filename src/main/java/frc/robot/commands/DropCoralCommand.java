package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHandler;

/**
 * Command that runs the coral intake in reverse until the game piece is released.
 * Automatically stops the intake when complete.
 */
public class DropCoralCommand extends Command {
    private final CoralHandler coralHandler;

    /**
     * Creates a new DropCoralCommand.
     */
    public DropCoralCommand() {
        coralHandler = CoralHandler.getInstance();
        addRequirements(coralHandler);
    }

    @Override
    public void initialize() {
        coralHandler.dropCoral();
    }

    @Override
    public void execute() {
        coralHandler.dropCoral();
    }

    @Override
    public boolean isFinished() {
        return !coralHandler.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        coralHandler.stopIntake();
    }
}
