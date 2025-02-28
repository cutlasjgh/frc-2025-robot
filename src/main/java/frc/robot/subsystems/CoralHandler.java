package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;

import java.util.ArrayList;

import com.ctre.phoenix.CANifier;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.subsubsytems.LimitSwitch;

import frc.robot.Constants.CoralConstants;
import frc.robot.subsubsytems.LimitedPIDSubsystem;
import frc.robot.subsubsytems.LimitedPIDSubsystem.SubsystemState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
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
     * Represents a configuration state of the arm mechanism.
     */
    private static class ArmState {
        public final Distance elevatorHeight;
        public final Angle armAngle;

        public ArmState(Distance elevatorHeight, Angle armAngle) {
            this.elevatorHeight = elevatorHeight;
            this.armAngle = armAngle;
        }
    }

    /**
     * Predefined positions for the end effector.
     */
    private enum HandlerPosition {
        GROUND(new Translation2d(CoralConstants.ARM_LENGTH.in(Inch), 0)),
        SCORE(new Translation2d(0, CoralConstants.ELEVATOR_HEIGHT.in(Inch))),
        CUSTOM(null);

        public final Translation2d target;

        private HandlerPosition(Translation2d target) {
            this.target = target;
        }
    }

    /** Elevator subsystem instance. */
    private final LimitedPIDSubsystem elevator;
    /** Arm subsystem instance. */
    private final LimitedPIDSubsystem arm;
    /** Intake motor controller. */
    private final SparkMax intakeMotor;
    /** Current position setting. */
    private HandlerPosition currentPosition = HandlerPosition.CUSTOM;
    /** Arm length in inches. */
    private final double armLength;
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

        armLength = CoralConstants.ARM_LENGTH.in(Inch);
        table = NetworkTableInstance.getDefault().getTable("CoralHandler");
    }
    
    /**
     * Creates and configures the elevator subsystem.
     */
    private LimitedPIDSubsystem createElevatorSubsystem() {
        // Create limit switches first
        LimitSwitch minLimitSwitch = LimitSwitch.createDIO(
            CoralConstants.ELEVATOR_BOTTOM_LIMIT_CHANNEL,
            null, null);
        
        LimitSwitch maxLimitSwitch = LimitSwitch.createCANifier(
            canifier,
            CoralConstants.ELEVATOR_TOP_LIMIT_PIN,
            null, null);
            
        // Then pass them to the subsystem constructor
        return new LimitedPIDSubsystem(
            CoralConstants.ELEVATOR_CAN_ID,
            CoralConstants.ELEVATOR_DISTANCE_PER_ROTATION.in(Inch),
            0,
            CoralConstants.ELEVATOR_HEIGHT.in(Inch),
            CoralConstants.ELEVATOR_PID,
            minLimitSwitch,
            maxLimitSwitch,
            CoralConstants.POSITION_TOLERANCE,
            LimitedPIDSubsystem.ControlMode.POSITION,
            CoralConstants.ELEVATOR_INVERTED,
            CoralConstants.ELEVATOR_ZEROING_POWER
        );
    }
    
    /**
     * Creates and configures the arm subsystem.
     */
    private LimitedPIDSubsystem createArmSubsystem() {
        // Create limit switches first
        LimitSwitch minLimitSwitch = LimitSwitch.createCANifier(
            canifier,
            CoralConstants.ARM_MIN_LIMIT_PIN,
            null, null);
            
        LimitSwitch maxLimitSwitch = LimitSwitch.createCANifier(
            canifier,
            CoralConstants.ARM_MAX_LIMIT_PIN,
            null, null);
            
        // Then pass them to the subsystem constructor
        return new LimitedPIDSubsystem(
            CoralConstants.ARM_CAN_ID,
            CoralConstants.ARM_ANGLE_PER_ROTATION.in(Degree),
            CoralConstants.ARM_MIN_ANGLE.in(Degree),
            CoralConstants.ARM_MAX_ANGLE.in(Degree),
            CoralConstants.ARM_PID,
            minLimitSwitch,
            maxLimitSwitch,
            CoralConstants.POSITION_TOLERANCE,
            LimitedPIDSubsystem.ControlMode.POSITION,
            CoralConstants.ARM_INVERTED,
            CoralConstants.ARM_ZEROING_POWER
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

    /**
     * Converts arm state to end effector position.
     */
    private Translation2d forwardKinematics(ArmState state) {
        double radians = Math.toRadians(state.armAngle.in(Degree));
        double x = armLength * Math.cos(radians);
        double y = state.elevatorHeight.in(Inch) + armLength * Math.sin(radians);
        return new Translation2d(x, y);
    }

    /**
     * Converts desired end effector position to arm state.
     * Returns null if position is unreachable.
     */
    private ArrayList<ArmState> inverseKinematics(Translation2d target) {
        ArrayList<Double> heightSolutions = new ArrayList<>();
        heightSolutions.add(target.getY() + Math.sqrt(Math.pow(armLength, 2) - Math.pow(target.getX(), 2)));
        heightSolutions.add(target.getY() - Math.sqrt(Math.pow(armLength, 2) - Math.pow(target.getX(), 2)));
        heightSolutions.removeIf(height -> height < 0 || height > CoralConstants.ELEVATOR_HEIGHT.in(Inch) || Double.isNaN(height));

        ArrayList<ArmState> validSolutions = new ArrayList<>();
        for (double height : heightSolutions) {
            double armAngle = Math.toDegrees(Math.atan2(target.getX(), target.getY() - height));
            if (armAngle >= CoralConstants.ARM_MIN_ANGLE.in(Degree) && armAngle <= CoralConstants.ARM_MAX_ANGLE.in(Degree)) {
                validSolutions.add(new ArmState(Inch.of(height), Degree.of(armAngle)));
            }
        }

        return validSolutions;
    }

    /**
     * Sets both elevator and arm to a predefined position.
     * <p>Example:
     * <pre>
     * {@code
     * CoralHandler.getInstance().setPosition(CoralHandler.HandlerPosition.SCORE);
     * }
     * </pre>
     */
    public void setPosition(HandlerPosition targetPosition) {
        if (currentPosition == targetPosition || 
            elevator.getState() == SubsystemState.UNKNOWN || 
            arm.getState() == SubsystemState.UNKNOWN) return;

        if (targetPosition != HandlerPosition.CUSTOM && targetPosition.target != null) {
            ArrayList<ArmState> solutions = inverseKinematics(targetPosition.target);
            if (!solutions.isEmpty()) {
                ArmState solution = solutions.get(0);
                elevator.set(solution.elevatorHeight.in(Inch));
                arm.set(solution.armAngle.in(Degree));
                currentPosition = targetPosition;
            }
        }
    }

    /**
     * Sets the end effector to a custom 2D position.
     * <p>Example:
     * <pre>
     * {@code
     * CoralHandler.getInstance().setCustomPosition(new Translation2d(15.0, 20.0));
     * }
     * </pre>
     */
    public void setCustomPosition(Translation2d target) {
        ArrayList<ArmState> solutions = inverseKinematics(target);
        if (!solutions.isEmpty()) {
            ArmState solution = solutions.get(0);
            elevator.set(solution.elevatorHeight.in(Inch));
            arm.set(solution.armAngle.in(Degree));
            currentPosition = HandlerPosition.CUSTOM;
        }
    }

    /**
     * Applies zeroing voltage to subsystems with unknown positions.
     * This helps bring mechanisms to their limit switches for position detection.
     * 
     * <p>Only applies voltage if a subsystem's position is currently unknown.
     * Once a limit switch is triggered, the position will be updated automatically.
     * <p>Example:
     * <pre>
     * {@code
     * CoralHandler.getInstance().zeroIfNeeded();
     * }
     * </pre>
     */
    public void zeroIfNeeded() {
        elevator.zeroIfNeeded();
        arm.zeroIfNeeded();
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
    }
}
