package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralArmConstants;
import frc.robot.Constants.CoralArmConstants.ArmState;
import frc.robot.helpers.LimitedPID;

import java.util.function.BooleanSupplier;

/**
 * Subsystem that controls the Coral Arm, responsible for moving the arm to
 * different positions.
 */
public class CoralArm extends SubsystemBase {
    private static CoralArm instance;

    public static CoralArm getInstance() {
        if (instance == null) {
            instance = new CoralArm();
        }
        return instance;
    }

    private final LimitedPID elevatorController = new LimitedPID(
            CoralArmConstants.ELEVATOR_CAN_ID, // CAN ID
            CoralArmConstants.ELEVATOR_DISTANCE_PER_ROTATION.in(Inch), // Conversion factor
            CoralArmConstants.ELEVATOR_MIN_POSITION.in(Inch), // Min position
            CoralArmConstants.ELEVATOR_MAX_POSITION.in(Inch), // Max position
            CoralArmConstants.ELEVATOR_PID, // PID constants
            CoralArmConstants.ELEVATOR_BOTTOM_LIMIT_CHANNEL, // Min limit channel
            CoralArmConstants.ELEVATOR_TOP_LIMIT_CHANNEL, // Max limit channel
            CoralArmConstants.POSITION_TOLERANCE, // Tolerance
            CoralArmConstants.ELEVATOR_INVERTED, // Inverted
            CoralArmConstants.ELEVATOR_RAMP_RATE, // Ramp rate
            CoralArmConstants.ELEVATOR_CURRENT_LIMIT // Current limit
    );
    private final LimitedPID elbowController = new LimitedPID(
            CoralArmConstants.ELBOW_CAN_ID, // CAN ID
            CoralArmConstants.ELBOW_ANGLE_PER_ROTATION.in(Degree), // Conversion factor
            CoralArmConstants.ELBOW_BACK_ANGLE.in(Degree), // Min position
            CoralArmConstants.ELBOW_FRONT_ANGLE.in(Degree), // Max position
            CoralArmConstants.ELBOW_PID, // PID constants
            CoralArmConstants.ELBOW_BACK_LIMIT_CHANNEL, // Min limit channel
            CoralArmConstants.ELBOW_FRONT_LIMIT_CHANNEL, // Max limit channel
            CoralArmConstants.POSITION_TOLERANCE, // Tolerance
            CoralArmConstants.ELBOW_INVERTED, // Inverted
            CoralArmConstants.ELBOW_RAMP_RATE, // Ramp rate
            CoralArmConstants.ELBOW_CURRENT_LIMIT // Current limit
    );
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Robot").getSubTable("CoralArm");

    private CoralArm() {
        // Initialize networktable entries
        table.getEntry("targetElbowAngle").setDouble(0.0);
        table.getEntry("targetElevatorHeight").setDouble(0.0);
        table.getEntry("currentSetpoint").setString("None");
    }

    /**
     * Trigger that is active when the elbow is on the front side.
     */
    public final Trigger onFront = new Trigger(() -> getCurrentElbowAngle().in(Degree) > 0);

    /**
     * Creates a command to set the arm position based on a predefined setpoint
     * 
     * @param setpointKey The key of the predefined setpoint in ARM_SETPOINTS
     * @return A command that moves the arm to the specified position
     */
    private Command setPosition(ArmState targetState) {
        BooleanSupplier isSwitchingSides = () -> {
            boolean currentSideIsFront = onFront.getAsBoolean();
            boolean targetSideIsFront = targetState.isFront();
            return currentSideIsFront != targetSideIsFront;
        };
        
        // Command that directly sets the target positions without any safety measures
        Command directMove = Commands.runOnce(() -> {
            elbowController.setPosition(targetState.elbowAngle().in(Degree));
            elevatorController.setPosition(targetState.elevatorHeight().in(Inch));
            table.getEntry("currentSetpoint").setString("Direct: " + targetState.elbowAngle().in(Degree) + "°, " + 
                targetState.elevatorHeight().in(Inch) + "\"");
        });
        
        // Safe transition command with upfront calculation of all waypoints
        Command safeTransition = Commands.sequence(
            // Initialize and calculate all setpoints at the beginning
            Commands.runOnce(() -> {
                table.getEntry("currentSetpoint").setString("Safe: " + targetState.elbowAngle().in(Degree) + "°, " + 
                    targetState.elevatorHeight().in(Inch) + "\"");
            }),
            
            // PHASE 1: Move to safe height and first intermediate angle simultaneously
            Commands.runOnce(() -> {
                // Get current side and set appropriate first safe angle
                boolean currentSideIsFront = onFront.getAsBoolean();
                double firstSafeAngle = currentSideIsFront ? 
                    CoralArmConstants.INTERMEDIATE_ELBOW_FRONT_ANGLE : 
                    CoralArmConstants.INTERMEDIATE_ELBOW_BACK_ANGLE;
                
                // Set both positions simultaneously
                elevatorController.setPosition(CoralArmConstants.SAFE_ELEVATOR_HEIGHT.in(Inch));
                elbowController.setPosition(firstSafeAngle);
                
                table.getEntry("phase").setString("Moving to safe height and first safe angle");
            }),
            
            // Wait until BOTH elevator AND elbow reach their targets
            Commands.waitUntil(() -> {
                boolean currentSideIsFront = onFront.getAsBoolean();
                double firstSafeAngle = currentSideIsFront ? 
                    CoralArmConstants.INTERMEDIATE_ELBOW_FRONT_ANGLE : 
                    CoralArmConstants.INTERMEDIATE_ELBOW_BACK_ANGLE;
                    
                double elevError = Math.abs(getCurrentElevatorHeight().in(Inch) - 
                                           CoralArmConstants.SAFE_ELEVATOR_HEIGHT.in(Inch));
                double elbowError = Math.abs(getCurrentElbowAngle().in(Degree) - firstSafeAngle);
                
                // Only proceed when both are within tolerance
                boolean elevatorReady = elevError < CoralArmConstants.POSITION_TOLERANCE;
                boolean elbowReady = elbowError < 5.0;
                
                return elevatorReady && elbowReady;
            }),
            
            // PHASE 2: Cross to other side by moving to the second intermediate angle
            Commands.runOnce(() -> {
                // Now move ONLY the elbow to the opposite side intermediate angle
                boolean targetSideIsFront = targetState.isFront();
                double secondSafeAngle = targetSideIsFront ? 
                    CoralArmConstants.INTERMEDIATE_ELBOW_FRONT_ANGLE : 
                    CoralArmConstants.INTERMEDIATE_ELBOW_BACK_ANGLE;
                
                elbowController.setPosition(secondSafeAngle);
                
                table.getEntry("phase").setString("Moving to second safe angle");
            }),
            
            // Wait until elbow crosses to opposite side
            Commands.waitUntil(() -> {
                boolean targetSideIsFront = targetState.isFront();
                double secondSafeAngle = targetSideIsFront ? 
                    CoralArmConstants.INTERMEDIATE_ELBOW_FRONT_ANGLE : 
                    CoralArmConstants.INTERMEDIATE_ELBOW_BACK_ANGLE;
                    
                double elbowError = Math.abs(getCurrentElbowAngle().in(Degree) - secondSafeAngle);
                
                return elbowError < 5.0;
            }),
            
            // PHASE 3: Now we're safe to move to final position
            Commands.runOnce(() -> {
                elbowController.setPosition(targetState.elbowAngle().in(Degree));
                elevatorController.setPosition(targetState.elevatorHeight().in(Inch));
                
                table.getEntry("phase").setString("Moving to final position");
            })
        );
        
        // Choose transition strategy based on whether we're switching sides
        return Commands.sequence(
            Commands.runOnce(() -> {
                table.getEntry("targetSetpoint").setString("Moving to: " + targetState.elbowAngle().in(Degree) + "°, " + 
                    targetState.elevatorHeight().in(Inch) + "\"");
            }),
            Commands.either(
                safeTransition,
                directMove,
                isSwitchingSides
            ),
            // Wait until positions are reached
            Commands.waitUntil(() -> {
                double elbowError = Math.abs(getCurrentElbowAngle().in(Degree) - 
                                            targetState.elbowAngle().in(Degree));
                double elevatorError = Math.abs(getCurrentElevatorHeight().in(Inch) - 
                                               targetState.elevatorHeight().in(Inch));
                
                boolean atPosition = elbowError < 5.0 && elevatorError < 1.0;
                table.getEntry("atTargetPosition").setBoolean(atPosition);
                
                return atPosition;
            }),
            
            Commands.runOnce(() -> {
                table.getEntry("lastCompletedSetpoint").setString(targetState.elbowAngle().in(Degree) + "°, " + 
                    targetState.elevatorHeight().in(Inch) + "\"");
                table.getEntry("phase").setString("Complete");
            })
        ).withName("setPosition");  // Removed timeout entirely
    }

    /**
     * Creates a command to set the arm position based on a predefined setpoint name
     * 
     * @param setpointKey The key of the predefined setpoint in ARM_SETPOINTS
     * @return A command that moves the arm to the specified position
     */
    public Command setPosition(String setpointKey) {
        ArmState state = CoralArmConstants.ARM_SETPOINTS.get(setpointKey);
        if (state == null) {
            return Commands.none().withName("SetPosition-Invalid");
        }
        return setPosition(state).withName("SetPosition-" + setpointKey);
    }

    /**
     * Command to move the arm to the ZERO position
     * @return Command to set the arm to the ZERO position
     */
    public Command setZero() {
        return setPosition("ZERO");
    }

    /**
     * Command to move the arm to the INTAKE position
     * @return Command to set the arm to the INTAKE position
     */
    public Command setIntake() {
        return setPosition("INTAKE");
    }

    /**
     * Command to move the arm to the LOW position
     * @return Command to set the arm to the LOW position
     */
    public Command setLow() {
        return setPosition("LOW");
    }

    /**
     * Command to move the arm to the MID position
     * @return Command to set the arm to the MID position
     */
    public Command setMid() {
        return setPosition("MID");
    }

    /**
     * Command to move the arm to the HIGH position
     * @return Command to set the arm to the HIGH position
     */
    public Command setHigh() {
        return setPosition("HIGH");
    }

    /**
     * Command to move the arm to the CLIMB position
     * @return Command to set the arm to the CLIMB position
     */
    public Command setClimb() {
        return setPosition("HIGH");
    }

    /**
     * Gets the current elbow angle
     * 
     * @return Current elbow angle
     */
    public Angle getCurrentElbowAngle() {
        return Degree.of(elbowController.getPosition());
    }

    /**
     * Gets the current elevator height
     * 
     * @return Current elevator height
     */
    public Distance getCurrentElevatorHeight() {
        return Inch.of(elevatorController.getPosition());
    }

    @Override
    public void periodic() {
        elevatorController.update();
        elbowController.update();

        // Publish current positions
        table.getEntry("elevatorHeight").setDouble(getCurrentElevatorHeight().in(Inch));
        table.getEntry("elbowAngle").setDouble(getCurrentElbowAngle().in(Degree));

        // Publish limit switch states
        table.getEntry("elevatorAtMinLimit").setBoolean(elevatorController.isAtMinLimit());
        table.getEntry("elevatorAtMaxLimit").setBoolean(elevatorController.isAtMaxLimit());
        table.getEntry("elbowAtMinLimit").setBoolean(elbowController.isAtMinLimit());
        table.getEntry("elbowAtMaxLimit").setBoolean(elbowController.isAtMaxLimit());

        // Publish subsystem states
        table.getEntry("elevatorStateKnown")
                .setBoolean(elevatorController.getState() == LimitedPID.SubsystemState.KNOWN);
        table.getEntry("elbowStateKnown").setBoolean(elbowController.getState() == LimitedPID.SubsystemState.KNOWN);

        // Essential motor outputs for monitoring
        table.getEntry("elbowMotorOutput").setDouble(elbowController.getMotor().get());
        table.getEntry("elevatorMotorOutput").setDouble(elevatorController.getMotor().get());
    }
}
