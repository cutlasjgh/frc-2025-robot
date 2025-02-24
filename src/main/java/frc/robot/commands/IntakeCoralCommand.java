package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHandler;

/**
 * Command that runs the coral intake until either a game piece is detected or the command is cancelled.
 * Automatically stops the intake when complete.
 */
public class IntakeCoralCommand extends Command {
    private final CoralHandler coralHandler;

    /**
     * Creates a new IntakeCoralCommand.
     */
    public IntakeCoralCommand() {
        coralHandler = CoralHandler.getInstance();
        addRequirements(coralHandler);
    }

    @Override
    public void initialize() {
        coralHandler.intakeCoral();
    }

    @Override
    public void execute() {
        coralHandler.intakeCoral();
    }

    @Override
    public boolean isFinished() {
        return coralHandler.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        coralHandler.stopIntake();
    }
}
