package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaArm;

public class IntakeAlga extends Command {

    public IntakeAlga() {
        addRequirements(AlgaArm.getInstance());
    }

    public void execute() {
        AlgaArm.getInstance().intakeAlga();
    }

    public boolean isFinished() {
        return AlgaArm.getInstance().hasAlga();
    }
}
