package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralHandler;
import frc.robot.subsystems.CoralHandler.HandlerPosition;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Degree;

public class SetCoralPosition extends SequentialCommandGroup {

    /**
     * Constructs a sequential command group that transitions the coral mechanism
     * to the target setpoint.
     * 
     * @param coralHandler   the CoralHandler subsystem
     * @param targetPosition the desired target position (using the enum defined in
     *                       CoralHandler)
     */
    public SetCoralPosition(CoralHandler coralHandler, HandlerPosition targetPosition) {
        // Add requirements
        addRequirements(coralHandler);
        
        // Start by setting the current status to CUSTOM.
        addCommands(new InstantCommand(() -> coralHandler.setCurrentPosition(HandlerPosition.CUSTOM), coralHandler));

        boolean switchingSides = coralHandler.isFront() != targetPosition.isFront();

        if (switchingSides) {
            double safeElevator = CoralConstants.SAFE_ELEVATOR_HEIGHT.in(Inch);
            boolean alreadyAbove = coralHandler.getElevator().getPosition() > safeElevator;
            double thisSideAngle = coralHandler.isFront()
                    ? CoralConstants.INTERMEDIATE_ARM_FRONT_ANGLE
                    : CoralConstants.INTERMEDIATE_ARM_BACK_ANGLE;

            double otherSideAngle = coralHandler.isFront()
                    ? CoralConstants.INTERMEDIATE_ARM_BACK_ANGLE
                    : CoralConstants.INTERMEDIATE_ARM_FRONT_ANGLE;

            // Step 1: If not already above safe height, move elevator up and arm to this side position
            if (!alreadyAbove) {
                addCommands(
                    new InstantCommand(() -> {
                        coralHandler.getElevator().set(safeElevator);
                        coralHandler.getArm().set(thisSideAngle);
                    }, coralHandler),
                    new WaitUntilCommand(() -> {
                        double elevError = Math.abs(coralHandler.getElevator().getPosition() - safeElevator);
                        double armError = Math.abs(coralHandler.getArm().getPosition() - thisSideAngle);
                        return elevError < 1.0 && armError < 5.0;
                    })
                );
            }
            
            // Step 2: Move arm to other side
            addCommands(
                new InstantCommand(() -> {
                    coralHandler.getArm().set(otherSideAngle);
                }, coralHandler),
                new WaitUntilCommand(() -> {
                    double armError = Math.abs(coralHandler.getArm().getPosition() - otherSideAngle);
                    return armError < 5.0;
                })
            );
        }

        // No side switch: command target positions immediately.
        addCommands(new InstantCommand(() -> {
            coralHandler.getElevator().set(targetPosition.elevatorHeight.in(Inch));
            coralHandler.getArm().set(targetPosition.armAngle.in(Degree));
        }, coralHandler));

        // Wait until final target positions are reached.
        addCommands(new WaitUntilCommand(() -> {
            double finalElevError = Math
                    .abs(coralHandler.getElevator().getPosition() - targetPosition.elevatorHeight.in(Inch));
            double finalArmError = Math.abs(coralHandler.getArm().getPosition() - targetPosition.armAngle.in(Degree));
            return finalElevError < 1.0 && finalArmError < 5.0;
        }));

        // Conclude by updating the current status to the requested target.
        addCommands(new InstantCommand(() -> coralHandler.setCurrentPosition(targetPosition), coralHandler));
    }
}
