package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralSuperstructureConstants;
import frc.robot.subsubsytems.LimitedPID;

public class CoralSuperstructure extends SubsystemBase {
    /**
     * Represents a specific position state of the arm
     */
    public record ArmState(Angle elbowAngle, Distance elevatorHeight) {
        public boolean isFront() {
            return elbowAngle.in(Degree) > 0;
        }
    }
    
    /**
     * Represents predefined arm positions
     */
    public enum ArmSetpoint {
        ZERO(new ArmState(CoralSuperstructureConstants.ELBOW_MAX_ANGLE, Inch.of(0.0))),
        INTAKE(new ArmState(Degree.of(70), Inch.of(0.0))),
        LOW(new ArmState(Degree.of(-90), Inch.of(5.0))),
        MID(new ArmState(Degree.of(-35), Inch.of(0.0))),
        HIGH(new ArmState(Degree.of(-35), Inch.of(15.0))),
        CLIMB(new ArmState(Degree.of(-90), Inch.of(13.5)));

        public final ArmState state;

        private ArmSetpoint(ArmState state) {
            this.state = state;
        }
    }
    
    /**
     * Represents the movement state of the arm
     */
    private enum MovementState {
        IDLE,
        MOVING_TO_SAFE_HEIGHT,
        CROSSING_SIDES,
        MOVING_TO_TARGET
    }
    
    // Tolerances
    private static final double ELEVATOR_TOLERANCE = 1.0;
    private static final double ELBOW_TOLERANCE = 5.0;
    private static final double POSITION_MATCH_TOLERANCE_ELBOW = 15.0;
    private static final double POSITION_MATCH_TOLERANCE_ELEVATOR = 2.0;

    // Singleton instance
    private static CoralSuperstructure instance;
    
    // Hardware components
    private final SparkMax coralMotor;
    private final DigitalInput coralSensor;
    private final LimitedPID elbowController;
    private final LimitedPID elevatorController;
    private final NetworkTable table;
    
    // State tracking
    private ArmState currentState;
    private ArmState targetState;
    private MovementState movementState = MovementState.IDLE;
    private boolean isEjecting = false;  // Track if we're in eject mode
    
    // Public triggers for sensors and status
    public final Trigger doesntHaveCoral;
    public final Trigger hasCoral;
    public final Trigger coralRunning;
    public final Trigger ejecting;
    
    /**
     * Gets the singleton instance of CoralSuperstructure
     */
    public static CoralSuperstructure getInstance() {
        if (instance == null) {
            instance = new CoralSuperstructure();
        }
        return instance;
    }
    
    /**
     * Private constructor for singleton
     */
    private CoralSuperstructure() {
        // Initialize hardware components
        coralMotor = new SparkMax(CoralSuperstructureConstants.MANIPULATOR_CAN_ID, MotorType.kBrushless);
        configureCoralMotor();
        
        coralSensor = new DigitalInput(CoralSuperstructureConstants.MANIPULATOR_SENSOR_CHANNEL);
        
        elbowController = new LimitedPID(
            CoralSuperstructureConstants.ELBOW_CAN_ID,
            CoralSuperstructureConstants.ELBOW_ANGLE_PER_ROTATION.in(Degree),
            CoralSuperstructureConstants.ELBOW_MIN_ANGLE.in(Degree),
            CoralSuperstructureConstants.ELBOW_MAX_ANGLE.in(Degree),
            CoralSuperstructureConstants.ELBOW_PID,
            CoralSuperstructureConstants.ELBOW_MIN_LIMIT_CHANNEL,
            CoralSuperstructureConstants.ELBOW_MAX_LIMIT_CHANNEL,
            CoralSuperstructureConstants.POSITION_TOLERANCE,
            LimitedPID.ControlMode.POSITION,
            CoralSuperstructureConstants.ELBOW_INVERTED,
            CoralSuperstructureConstants.ELBOW_RAMP_RATE,
            CoralSuperstructureConstants.ELBOW_CURRENT_LIMIT
        );
        
        elevatorController = new LimitedPID(
            CoralSuperstructureConstants.ELEVATOR_CAN_ID,
            CoralSuperstructureConstants.ELEVATOR_DISTANCE_PER_ROTATION.in(Inch),
            0,
            CoralSuperstructureConstants.ELEVATOR_HEIGHT.in(Inch),
            CoralSuperstructureConstants.ELEVATOR_PID,
            CoralSuperstructureConstants.ELEVATOR_BOTTOM_LIMIT_CHANNEL,
            CoralSuperstructureConstants.ELEVATOR_TOP_LIMIT_CHANNEL,
            CoralSuperstructureConstants.POSITION_TOLERANCE,
            LimitedPID.ControlMode.POSITION,
            CoralSuperstructureConstants.ELEVATOR_INVERTED,
            CoralSuperstructureConstants.ELEVATOR_RAMP_RATE,
            CoralSuperstructureConstants.ELEVATOR_CURRENT_LIMIT
        );
        
        table = NetworkTableInstance.getDefault().getTable("CoralSuperstructure");
        
        // Initialize state
        updateCurrentState();
        
        // Create triggers
        doesntHaveCoral = new Trigger(() -> coralSensor.get());
        hasCoral = new Trigger(() -> !coralSensor.get());
        coralRunning = new Trigger(() -> coralMotor.get() != 0);
        ejecting = new Trigger(() -> isEjecting);
        
        // Configure behaviors
        configureAutoBehaviors();
        
        // Set default command for position control
        setDefaultCommand(run(this::updateMovement).withName("PositionControl"));
    }
    
    /**
     * Configures the coral motor controller
     */
    private void configureCoralMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(CoralSuperstructureConstants.MANIPULATOR_INVERTED);
        config.smartCurrentLimit(20);
        config.openLoopRampRate(0.1);
        config.closedLoopRampRate(0.1);
        coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    
    /**
     * Sets up auto behaviors and event-driven actions
     */
    private void configureAutoBehaviors() {
        // Auto transition to MID when coral is detected during intake
        hasCoral.onTrue(Commands.sequence(
            // First stop the intake motor before moving
            Commands.runOnce(this::stopIntake),
            // Then transition to MID position if we're at INTAKE
            Commands.runOnce(() -> {
                if (isAtSetpoint(ArmSetpoint.INTAKE)) {
                    goToSetpoint(ArmSetpoint.MID);
                }
            })
        ));
        
        // Combined trigger for auto-intake and auto-stop
        Trigger atIntakePosition = new Trigger(() -> isAtSetpoint(ArmSetpoint.INTAKE));
        
        // Auto start intake when at INTAKE position without coral
        atIntakePosition.and(doesntHaveCoral).and(coralRunning.negate())
            .onTrue(Commands.runOnce(this::startIntake));
            
        // Auto stop intake when leaving INTAKE position
        atIntakePosition.negate().and(coralRunning)
            .onTrue(Commands.runOnce(() -> {
                // Only stop if we're currently running in intake mode
                if (isIntaking()) {
                    stopIntake();
                }
            }));
            
        // Auto stop eject when leaving eject-capable position
        Trigger atEjectPosition = new Trigger(this::isInDropPosition);
        atEjectPosition.negate().and(ejecting)
            .onTrue(Commands.runOnce(() -> {
                stopEject();
            }));
    }
    
    /**
     * Updates the current state from sensor readings
     */
    private void updateCurrentState() {
        currentState = new ArmState(
            Degree.of(elbowController.getPosition()),
            Inch.of(elevatorController.getPosition())
        );
    }
    
    /**
     * Checks if a value is within tolerance of a target
     */
    private boolean isWithinTolerance(double value, double target, double tolerance) {
        return Math.abs(value - target) < tolerance;
    }
    
    /**
     * Checks if the arm is at the specified setpoint
     */
    private boolean isAtSetpoint(ArmSetpoint setpoint) {
        if (currentState == null) return false;
        
        return isWithinTolerance(
            currentState.elbowAngle().in(Degree),
            setpoint.state.elbowAngle().in(Degree),
            POSITION_MATCH_TOLERANCE_ELBOW
        ) && isWithinTolerance(
            currentState.elevatorHeight().in(Inch),
            setpoint.state.elevatorHeight().in(Inch),
            POSITION_MATCH_TOLERANCE_ELEVATOR
        );
    }
    
    /**
     * Checks if the arm is in a position where dropping coral is allowed
     */
    private boolean isInDropPosition() {
        return isAtSetpoint(ArmSetpoint.HIGH) || 
               isAtSetpoint(ArmSetpoint.MID) || 
               isAtSetpoint(ArmSetpoint.LOW);
    }
    
    /**
     * Gets the intermediate angle for a side
     */
    private double getIntermediateAngle(boolean forFrontSide) {
        return forFrontSide ? 
            CoralSuperstructureConstants.INTERMEDIATE_ELBOW_FRONT_ANGLE : 
            CoralSuperstructureConstants.INTERMEDIATE_ELBOW_BACK_ANGLE;
    }
    
    /**
     * Gets the name of the current position
     */
    private String getCurrentPositionName() {
        for (ArmSetpoint setpoint : ArmSetpoint.values()) {
            if (isAtSetpoint(setpoint)) {
                return setpoint.name();
            }
        }
        return "CUSTOM";
    }
    
    /**
     * Sets the arm to move to a specific arm state
     */
    private void goTo(ArmState target) {
        if (target == null) return;
        
        targetState = target;
        updateCurrentState();
        
        // Determine if we need to switch sides
        boolean switchingSides = currentState.isFront() != target.isFront();
        
        if (switchingSides) {
            // Start with moving to safety position
            movementState = MovementState.MOVING_TO_SAFE_HEIGHT;
            
            // Move to safe position on current side
            elevatorController.set(CoralSuperstructureConstants.SAFE_ELEVATOR_HEIGHT.in(Inch));
            elbowController.set(getIntermediateAngle(currentState.isFront()));
        } else {
            // Direct movement to target
            movementState = MovementState.MOVING_TO_TARGET;
            elevatorController.set(target.elevatorHeight().in(Inch));
            elbowController.set(target.elbowAngle().in(Degree));
        }
    }
    
    /**
     * Sets the arm to move to a predefined setpoint with safety checks
     */
    public void goToSetpoint(ArmSetpoint setpoint) {
        // Safety check: Don't allow moving to INTAKE if we already have coral
        if (setpoint == ArmSetpoint.INTAKE && hasCoral.getAsBoolean()) {
            // Log the safety prevention
            System.out.println("Prevented moving to INTAKE while coral is present");
            return;
        }
        
        goTo(setpoint.state);
    }
    
    /**
     * Creates a command to go to a setpoint with safety checks
     */
    public Command setpointCommand(ArmSetpoint setpoint) {
        return runOnce(() -> {
            // For INTAKE position, only execute if we don't have coral already
            if (setpoint == ArmSetpoint.INTAKE) {
                if (!hasCoral.getAsBoolean()) {
                    goToSetpoint(setpoint);
                } else {
                    // Could add user feedback here (LEDs, rumble, etc.)
                    System.out.println("Cannot move to INTAKE: coral already present");
                }
            } else {
                // For all other positions, proceed normally
                goToSetpoint(setpoint);
            }
        }).withName("Set" + setpoint.name());
    }
    
    /**
     * Updates the arm movement state machine
     */
    private void updateMovement() {
        if (movementState == MovementState.IDLE || targetState == null) {
            return; // Nothing to do
        }

        updateCurrentState();
        double elevatorPosition = elevatorController.getPosition();
        double elbowPosition = elbowController.getPosition();

        switch (movementState) {
            case IDLE:
                break;
                
            case MOVING_TO_SAFE_HEIGHT:
                // Check if at safety position
                if (isWithinTolerance(
                        elevatorPosition,
                        CoralSuperstructureConstants.SAFE_ELEVATOR_HEIGHT.in(Inch), 
                        ELEVATOR_TOLERANCE) &&
                    isWithinTolerance(
                        elbowPosition,
                        getIntermediateAngle(currentState.isFront()),
                        ELBOW_TOLERANCE)) {
                    
                    // Move to crossing phase
                    movementState = MovementState.CROSSING_SIDES;
                    elbowController.set(getIntermediateAngle(targetState.isFront()));
                }
                break;
                
            case CROSSING_SIDES:
                // Check if crossed to target side
                if (isWithinTolerance(
                        elbowPosition,
                        getIntermediateAngle(targetState.isFront()),
                        ELBOW_TOLERANCE)) {
                    
                    // Move to final position
                    movementState = MovementState.MOVING_TO_TARGET;
                    elevatorController.set(targetState.elevatorHeight().in(Inch));
                    elbowController.set(targetState.elbowAngle().in(Degree));
                }
                break;
                
            case MOVING_TO_TARGET:
                // Check if at target position
                if (isWithinTolerance(
                        elevatorPosition,
                        targetState.elevatorHeight().in(Inch),
                        ELEVATOR_TOLERANCE) &&
                    isWithinTolerance(
                        elbowPosition,
                        targetState.elbowAngle().in(Degree),
                        ELBOW_TOLERANCE)) {
                    
                    movementState = MovementState.IDLE;
                }
                break;
        }
    }
    
    // Public setpoint command methods
    public Command setZero() { return setpointCommand(ArmSetpoint.ZERO); }
    public Command setIntake() { return setpointCommand(ArmSetpoint.INTAKE); }
    public Command setLow() { return setpointCommand(ArmSetpoint.LOW); }
    public Command setMid() { return setpointCommand(ArmSetpoint.MID); }
    public Command setHigh() { return setpointCommand(ArmSetpoint.HIGH); }
    public Command setClimb() { return setpointCommand(ArmSetpoint.CLIMB); }
    
    /**
     * Directly set the coral motor to intake power, without using commands
     * This ensures we don't interfere with the command system
     */
    private void startIntake() {
        coralMotor.set(CoralSuperstructureConstants.INTAKE_POWER);
    }
    
    /**
     * Directly stop the coral motor, without using commands
     * This ensures we don't interfere with the command system
     */
    private void stopIntake() {
        coralMotor.set(0);
    }
    
    /**
     * Start ejecting mode - runs motor at high power
     */
    private void startEject() {
        isEjecting = true;
        coralMotor.set(CoralSuperstructureConstants.EJECT_POWER);
    }
    
    /**
     * Stop ejecting mode
     */
    private void stopEject() {
        isEjecting = false;
        coralMotor.set(0);
    }
    
    /**
     * Command to intake coral (only in intake position)
     */
    public Command intake() {
        return Commands.either(
            run(() -> startIntake())
                .onlyWhile(doesntHaveCoral)
                .finallyDo((interrupted) -> stopIntake())
                .withName("coralIntake"),
            Commands.none(),
            () -> isAtSetpoint(ArmSetpoint.INTAKE)
        );
    }
    
    /**
     * Command to drop coral (only in drop positions)
     */
    public Command drop() {
        return Commands.either(
            run(() -> coralMotor.set(CoralSuperstructureConstants.DROP_POWER))
                .onlyWhile(hasCoral)
                .finallyDo((interrupted) -> stopIntake())
                .withName("coralDrop"),
            Commands.none(),
            this::isInDropPosition
        );
    }
    
    /**
     * Command to eject (force out) any stuck objects (only in drop positions)
     */
    public Command eject() {
        return Commands.either(
            runOnce(() -> startEject())
                .withName("coralEject"),
            Commands.none(),
            this::isInDropPosition
        );
    }
    
    /**
     * Command to stop coral motor
     */
    public Command stop() {
        return runOnce(() -> stopIntake()).withName("coralStop");
    }
    
    /**
     * Command to toggle coral intake/drop/eject based on position and sensor status
     */
    public Command toggle() {
        return runOnce(() -> {
            if (coralRunning.getAsBoolean()) {
                // If motor is running, always stop first
                if (isEjecting) {
                    stopEject();
                } else {
                    stop().schedule();
                }
            } else {
                // Motor is stopped, determine appropriate action
                if (hasCoral.getAsBoolean() && isInDropPosition()) {
                    // If we have coral and are in drop position, drop it
                    drop().schedule();
                } else if (doesntHaveCoral.getAsBoolean() && isAtSetpoint(ArmSetpoint.INTAKE)) {
                    // If we don't have coral and are in intake position, intake
                    intake().schedule();
                } else if (doesntHaveCoral.getAsBoolean() && isInDropPosition()) {
                    // If we don't have coral but are in a drop position, eject mode
                    startEject();
                }
            }
        }).withName("coralToggle");
    }
    
    /**
     * Command to zero the arm through limit switches
     */
    public Command zeroCommand() {
        return Commands.sequence(
            // Stop current operations
            Commands.runOnce(() -> {
                stop().schedule();
                movementState = MovementState.IDLE;
            }),
            
            // Step 1: Elevator to top limit
            Commands.runOnce(() -> 
                elevatorController.getMotor().set(CoralSuperstructureConstants.ELEVATOR_ZEROING_POWER_UP)),
            Commands.waitUntil(elevatorController::isAtMaxLimit),
            Commands.runOnce(() -> elevatorController.getMotor().set(0)),
            
            // Step 2: Elbow to max limit
            Commands.runOnce(() -> 
                elbowController.getMotor().set(CoralSuperstructureConstants.ELBOW_ZEROING_POWER)),
            Commands.waitUntil(elbowController::isAtMaxLimit),
            Commands.runOnce(() -> elbowController.getMotor().set(0)),
            
            // Step 3: Elevator to bottom limit
            Commands.runOnce(() -> 
                elevatorController.getMotor().set(CoralSuperstructureConstants.ELEVATOR_ZEROING_POWER_DOWN)),
            Commands.waitUntil(elevatorController::isAtMinLimit),
            Commands.runOnce(() -> elevatorController.getMotor().set(0)),
            
            // Step 4: Update state and go to zero
            Commands.runOnce(() -> {
                updateCurrentState();
                goToSetpoint(ArmSetpoint.ZERO);
            })
        ).withName("ZeroCoralCommand");
    }
    
    @Override
    public void periodic() {
        updateCurrentState();
        
        // Update NetworkTables
        table.getEntry("coralRunning").setBoolean(coralRunning.getAsBoolean());
        table.getEntry("hasCoral").setBoolean(hasCoral.getAsBoolean());
        table.getEntry("isEjecting").setBoolean(isEjecting);
        table.getEntry("currentElbowAngle").setDouble(currentState.elbowAngle().in(Degree));
        table.getEntry("currentElevatorHeight").setDouble(currentState.elevatorHeight().in(Inch));
        table.getEntry("movementState").setString(movementState.toString());
        table.getEntry("currentPosition").setString(getCurrentPositionName());
        table.getEntry("canIntake").setBoolean(isAtSetpoint(ArmSetpoint.INTAKE) && doesntHaveCoral.getAsBoolean());
        table.getEntry("canDrop").setBoolean(isInDropPosition() && hasCoral.getAsBoolean());
        table.getEntry("canEject").setBoolean(isInDropPosition() && doesntHaveCoral.getAsBoolean());
        table.getEntry("elevatorAtMin").setBoolean(elevatorController.isAtMinLimit());
        table.getEntry("elevatorAtMax").setBoolean(elevatorController.isAtMaxLimit());
        table.getEntry("elbowAtMin").setBoolean(elbowController.isAtMinLimit());
        table.getEntry("elbowAtMax").setBoolean(elbowController.isAtMaxLimit());
        
        // Add min/max limits to NetworkTables
        table.getEntry("minElbowAngle").setDouble(CoralSuperstructureConstants.ELBOW_MIN_ANGLE.in(Degree));
        table.getEntry("maxElbowAngle").setDouble(CoralSuperstructureConstants.ELBOW_MAX_ANGLE.in(Degree));
        table.getEntry("minElevatorHeight").setDouble(0.0); // Minimum elevator height is always 0
        table.getEntry("maxElevatorHeight").setDouble(CoralSuperstructureConstants.ELEVATOR_HEIGHT.in(Inch));
        
        if (targetState != null) {
            table.getEntry("targetElbowAngle").setDouble(targetState.elbowAngle().in(Degree));
            table.getEntry("targetElevatorHeight").setDouble(targetState.elevatorHeight().in(Inch));
        }
    }
    
    /**
     * Checks if the coral motor is currently running in intake mode
     */
    private boolean isIntaking() {
        double speed = coralMotor.get();
        return speed > 0 && speed <= CoralSuperstructureConstants.INTAKE_POWER + 0.01;
    }
}
