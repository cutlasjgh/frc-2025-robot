package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaArm;

/**
 * Command that runs the alga intake until either a game piece is detected or the command is cancelled.
 * Automatically stops the intake when complete.
 */
public class IntakeAlgaCommand extends Command {
    private final AlgaArm algaArm;

    /**
     * Creates a new IntakeAlgaCommand.
     */
    public IntakeAlgaCommand() {
        algaArm = AlgaArm.getInstance();
        addRequirements(algaArm);
    }

    @Override
    public void initialize() {
        algaArm.startIntake();
    }

    @Override
    public void execute() {
        algaArm.startIntake();
    }

    @Override
    public boolean isFinished() {
        return algaArm.hasGamepiece();
    }

    @Override
    public void end(boolean interrupted) {
        algaArm.stopIntake();
    }
}
