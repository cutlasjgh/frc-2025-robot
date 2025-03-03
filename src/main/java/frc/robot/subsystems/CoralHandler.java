package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;

import com.ctre.phoenix.CANifier;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.subsubsytems.LimitSwitch;

import frc.robot.Constants.CoralConstants;
import frc.robot.subsubsytems.LimitedPID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Controls the Coral mechanism, a multi-joint manipulator consisting of an elevator, arm, and intake.
 * This subsystem manages coordinated movement between components and provides preset positions
 * for common operations.
 * 
 * <p>Features include:
 * <ul>
 *   <li>Position-controlled elevator and arm using encoders and limit switches
 *   <li>Automatic position detection using limit switches
 *   <li>Predefined positions for common operations (ground pickup, scoring)
 *   <li>Safety features including position limits and state validation
 * </ul>
 * 
 * <p>The subsystem uses REV Robotics SparkMax controllers with closed-loop position control
 * and hardware limit switches for position detection and safety.
 */
public class CoralHandler extends SubsystemBase {
    private static CoralHandler instance;

    /**
     * Predefined positions for the end effector.
     */
    // Updated HandlerPosition enum with only ZERO and CUSTOM
    public enum HandlerPosition {
        ZERO(CoralConstants.ARM_MAX_ANGLE, Inch.of(0.0)),
        INTAKE(Degree.of(70), Inch.of(0.0)),
        LOW(Degree.of(-90), Inch.of(5.0)),
        MID(Degree.of(-35), Inch.of(0.0)),
        HIGH(Degree.of(-35), Inch.of(15.0)),
        CLIMB(Degree.of(-90), Inch.of(13.5)),
        CUSTOM(null, null);

        public final Angle armAngle;
        public final Distance elevatorHeight;

        private HandlerPosition(Angle armAngle, Distance elevatorHeight) {
            this.armAngle = armAngle;
            this.elevatorHeight = elevatorHeight;
        }

        public boolean isFront() {
            return armAngle.in(Degree) > 0;
        }
    }

    /** Elevator subsystem instance. */
    private final LimitedPID elevator;
    /** Arm subsystem instance. */
    private final LimitedPID arm;
    /** Intake motor controller. */
    private final SparkMax intakeMotor;
    /** Current position setting. */
    private HandlerPosition currentPosition = HandlerPosition.CUSTOM;
    /** NetworkTable instance for publishing data. */
    private final NetworkTable table;

    private final CANifier canifier;

    /**
     * @return Singleton instance of the CoralHandler
     */
    public static CoralHandler getInstance() {
        if (instance == null) {
            instance = new CoralHandler();
        }
        return instance;
    }
    
    /**
     * Creates a new CoralHandler, initializing all subsystems.
     */
    public CoralHandler() {
        // Initialize CANifier
        canifier = new CANifier(CoralConstants.CANIFIER_ID);
        canifier.configFactoryDefault();

        // Initialize elevator with mixed limit switch types (DIO for min, CANifier for max)
        elevator = createElevatorSubsystem();
        
        // Initialize arm with CANifier limit switches
        arm = createArmSubsystem();

        // Initialize intake motor
        intakeMotor = createIntakeMotor();

        table = NetworkTableInstance.getDefault().getTable("CoralHandler");
    }
    
    /**
     * Creates and configures the elevator subsystem.
     */
    private LimitedPID createElevatorSubsystem() {
        // Then pass them to the subsystem constructor with additional parameters
        return new LimitedPID(
            CoralConstants.ELEVATOR_CAN_ID,
            CoralConstants.ELEVATOR_DISTANCE_PER_ROTATION.in(Inch),
            0,
            CoralConstants.ELEVATOR_HEIGHT.in(Inch),
            CoralConstants.ELEVATOR_PID,
            CoralConstants.ELEVATOR_BOTTOM_LIMIT_CHANNEL,
            CoralConstants.ELEVATOR_TOP_LIMIT_CHANNEL,
            CoralConstants.POSITION_TOLERANCE,
            LimitedPID.ControlMode.POSITION,
            CoralConstants.ELEVATOR_INVERTED,
            CoralConstants.ELEVATOR_RAMP_RATE,
            CoralConstants.ELEVATOR_CURRENT_LIMIT
        );
    }
    
    /**
     * Creates and configures the arm subsystem.
     */
    private LimitedPID createArmSubsystem() { 
        // Then pass them to the subsystem constructor with additional parameters
        return new LimitedPID(
            CoralConstants.ARM_CAN_ID,
            CoralConstants.ARM_ANGLE_PER_ROTATION.in(Degree),
            CoralConstants.ARM_MIN_ANGLE.in(Degree),
            CoralConstants.ARM_MAX_ANGLE.in(Degree),
            CoralConstants.ARM_PID,
            CoralConstants.ARM_MIN_LIMIT_CHANNEL,
            CoralConstants.ARM_MAX_LIMIT_CHANNEL,
            CoralConstants.POSITION_TOLERANCE,
            LimitedPID.ControlMode.POSITION,
            CoralConstants.ARM_INVERTED,
            CoralConstants.ARM_RAMP_RATE,
            CoralConstants.ARM_CURRENT_LIMIT
        );
    }
    
    /**
     * Creates and configures the intake motor.
     */
    private SparkMax createIntakeMotor() {
        SparkMax motor = new SparkMax(CoralConstants.INTAKE_CAN_ID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(CoralConstants.INTAKE_INVERTED);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        return motor;
    }
    
    public LimitedPID getElevator() {
        return elevator;
    }
    
    public LimitedPID getArm() {
        return arm;
    }
    
    // Expose current side based on motor position; positive indicates FRONT, negative indicates BACK.
    public boolean isFront() {
        return getArm().getPosition() > 0;
    }

    // Add public setter method for current position
    public void setCurrentPosition(HandlerPosition newPos) {
        currentPosition = newPos;
    }

    /**
     * Activates the intake motor to collect game pieces.
     * <p>Example:
     * <pre>
     * {@code
     * CoralHandler.getInstance().intakeCoral();
     * }
     * </pre>
     */
    public void intakeCoral() {
        intakeMotor.set(CoralConstants.INTAKE_POWER);
    }

    /**
     * Runs the intake in reverse to release the coral.
     * Continues until coral is no longer detected.
     * <p>Example:
     * <pre>
     * {@code
     * CoralHandler.getInstance().dropCoral();
     * }
     * </pre>
     */
    public void dropCoral() {
        intakeMotor.set(CoralConstants.OUTTAKE_POWER);
    }

    /**
     * Stops the intake motor's operation.
     * <p>Example:
     * <pre>
     * {@code
     * CoralHandler.getInstance().stopIntake();
     * }
     * </pre>
     */
    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    /**
     * Checks if the mechanism currently has a game piece.
     * Uses CANifier input to detect coral presence.
     * 
     * @return true if a coral is detected, false otherwise
     * <p>Example:
     * <pre>
     * {@code
     * boolean detected = CoralHandler.getInstance().hasCoral();
     * }
     * </pre>
     */
    public boolean hasCoral() {
        return !canifier.getGeneralInput(CoralConstants.INTAKE_SENSOR_PIN);
    }

    public HandlerPosition getPosition() {
        return currentPosition;
    }

    @Override
    public void periodic() {
        handleIntakeStateLogic();
        publishTelemetry();
    }
    
    /**
     * Handles the auto-stopping logic for the intake motor based on coral detection.
     */
    private void handleIntakeStateLogic() {
        boolean hasCoralNow = hasCoral();
        double motorPower = intakeMotor.get();
        
        // Stop intake if coral is detected during intake
        boolean shouldStopIntake = (hasCoralNow && motorPower > 0);
        
        // Stop outtake if coral is no longer detected during outtake
        boolean shouldStopOuttake = (!hasCoralNow && motorPower < 0);
        
        if (shouldStopIntake || shouldStopOuttake) {
            stopIntake();
        }
    }
    
    /**
     * Publishes telemetry data to NetworkTables.
     */
    private void publishTelemetry() {
        table.getEntry("hasCoral").setBoolean(hasCoral());
        table.getEntry("elevatorHeight").setDouble(elevator.getPosition());
        table.getEntry("armAngle").setDouble(arm.getPosition());
        table.getEntry("elevatorState").setString(elevator.getState().toString());
        table.getEntry("armState").setString(arm.getState().toString());
        
        // Add limit switch states to telemetry
        table.getEntry("elevatorMinLimit").setBoolean(elevator.isAtMinLimit());
        table.getEntry("elevatorMaxLimit").setBoolean(elevator.isAtMaxLimit());
        table.getEntry("armMinLimit").setBoolean(arm.isAtMinLimit());
        table.getEntry("armMaxLimit").setBoolean(arm.isAtMaxLimit());

        // Add current position to telemetry
        table.getEntry("currentPosition").setString(currentPosition.toString());
        table.getEntry("currentlyInFront").setBoolean(isFront());
    }
}
