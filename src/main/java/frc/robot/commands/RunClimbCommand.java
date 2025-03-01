package frc.robot.commands;

import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;

public class RunClimbCommand extends Command {
    private final Climb climb;

    public RunClimbCommand(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.runClimber();
    }

    @Override
    public void execute() {
        if (!climb.isAtBottom()) {
            climb.runClimber();
        } else {
            climb.stop();
        }
    }

    @Override
    public boolean isFinished() {
        // Always run until button is released
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }
}
