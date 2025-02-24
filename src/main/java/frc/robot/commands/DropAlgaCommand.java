package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaArm;

/**
 * Command that runs the alga intake in reverse until the game piece is released.
 * Automatically stops the intake when complete.
 */
public class DropAlgaCommand extends Command {
    private final AlgaArm algaArm;

    /**
     * Creates a new DropAlgaCommand.
     */
    public DropAlgaCommand() {
        algaArm = AlgaArm.getInstance();
        addRequirements(algaArm);
    }

    @Override
    public void initialize() {
        algaArm.dropGamepiece();
    }

    @Override
    public void execute() {
        algaArm.dropGamepiece();
    }

    @Override
    public boolean isFinished() {
        return !algaArm.hasGamepiece();
    }

    @Override
    public void end(boolean interrupted) {
        algaArm.stopIntake();
    }
}
