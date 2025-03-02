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
            
            // Safety first: If not already above safe height, raise elevator before arm movement
            if (!alreadyAbove) {
                addCommands(
                    new InstantCommand(() -> coralHandler.getElevator().set(safeElevator), coralHandler),
                    // Only wait for elevator to reach safe height before allowing arm to cross midpoint
                    new WaitUntilCommand(() -> {
                        double elevError = Math.abs(coralHandler.getElevator().getPosition() - safeElevator);
                        return elevError < 1.0;
                    })
                );
            }
            
            // Now we can directly set final position - arm will move continuously through the midpoint
            // but only after the elevator is at a safe height
            addCommands(new InstantCommand(() -> {
                coralHandler.getElevator().set(targetPosition.elevatorHeight.in(Inch));
                coralHandler.getArm().set(targetPosition.armAngle.in(Degree));
            }, coralHandler));
        } else {
            // No side switch: command target positions immediately
            addCommands(new InstantCommand(() -> {
                coralHandler.getElevator().set(targetPosition.elevatorHeight.in(Inch));
                coralHandler.getArm().set(targetPosition.armAngle.in(Degree));
            }, coralHandler));
        }

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
