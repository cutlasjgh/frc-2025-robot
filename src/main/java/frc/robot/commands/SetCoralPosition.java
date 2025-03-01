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
     * @param coralHandler the CoralHandler subsystem
     * @param targetPosition the desired target position (using the enum defined in CoralHandler)
     */
    public SetCoralPosition(CoralHandler coralHandler, HandlerPosition targetPosition) {
        // Start by setting the current status to CUSTOM.
        addCommands(new InstantCommand(() -> coralHandler.setCurrentPosition(HandlerPosition.CUSTOM), coralHandler));
        
        boolean switchingSides = coralHandler.getCurrentSide() != targetPosition.side;
        System.out.println("Switching sides: " + switchingSides);
        System.out.println("Current side: " + coralHandler.getCurrentSide());
        System.out.println("Target side: " + targetPosition.side);
        
        if (switchingSides) {
            double safeElevator = CoralConstants.SAFE_ELEVATOR_HEIGHT.in(Inch);
            boolean alreadyAbove = coralHandler.getElevator().getPosition() > safeElevator;
            // Determine flipped intermediate arm angle for over-the-top switch.
            double flippedArm = (coralHandler.getCurrentSide() == CoralHandler.Side.FRONT) ?
                CoralConstants.INTERMEDIATE_ARM_BACK_ANGLE :
                CoralConstants.INTERMEDIATE_ARM_FRONT_ANGLE;
            
            if (alreadyAbove) {
                addCommands(
                    // Only adjust the arm first.
                    new InstantCommand(() -> {
                        coralHandler.getArm().set(flippedArm);
                    }, coralHandler),
                    new WaitUntilCommand(() -> {
                        double armError = Math.abs(coralHandler.getArm().getPosition() - flippedArm);
                        return armError < 5.0;
                    }),
                    // Then lower elevator and set final arm position.
                    new InstantCommand(() -> {
                        coralHandler.getElevator().set(targetPosition.elevatorHeight.in(Inch));
                        coralHandler.getArm().set(targetPosition.armAngle.in(Degree));
                    }, coralHandler)
                );
            } else {
                addCommands(
                    // 1. Command intermediate setpoints for switching over the top.
                    new InstantCommand(() -> {
                        coralHandler.getElevator().set(safeElevator);
                        coralHandler.getArm().set(flippedArm);
                    }, coralHandler),
                    // 2. Wait until the intermediate positions are reached.
                    new WaitUntilCommand(() -> {
                        double elevError = Math.abs(coralHandler.getElevator().getPosition() - safeElevator);
                        double armError = Math.abs(coralHandler.getArm().getPosition() - flippedArm);
                        return elevError < 1.0 && armError < 5.0;
                    }),
                    // 3. Command the final target positions.
                    new InstantCommand(() -> {
                        coralHandler.getElevator().set(targetPosition.elevatorHeight.in(Inch));
                        coralHandler.getArm().set(targetPosition.armAngle.in(Degree));
                    }, coralHandler)
                );
            }
        } else {
            // No side switch: command target positions immediately.
            addCommands(new InstantCommand(() -> {
                coralHandler.getElevator().set(targetPosition.elevatorHeight.in(Inch));
                coralHandler.getArm().set(targetPosition.armAngle.in(Degree));
            }, coralHandler));
        }
        
        // 4. Wait until final target positions are reached.
        addCommands(new WaitUntilCommand(() -> {
            double finalElevError = Math.abs(coralHandler.getElevator().getPosition() - targetPosition.elevatorHeight.in(Inch));
            double finalArmError = Math.abs(coralHandler.getArm().getPosition() - targetPosition.armAngle.in(Degree));
            return finalElevError < 1.0 && finalArmError < 5.0;
        }));
        
        // Conclude by updating the current status to the requested target.
        addCommands(new InstantCommand(() -> coralHandler.setCurrentPosition(targetPosition), coralHandler));
    }
}
