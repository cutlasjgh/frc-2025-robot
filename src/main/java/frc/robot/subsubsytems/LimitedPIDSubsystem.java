package frc.robot.subsubsytems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.PID;

/**
 * Represents a PID-controlled subsystem with limit switches and position control.
 *
 * <p>Example usage:
 * <pre>
 * {@code
 * // Assume minSwitch and maxSwitch are created previously:
 * LimitSwitch minSwitch = LimitSwitch.createDIO(0, 
 *         () -> System.out.println("Min limit triggered"), null);
 * LimitSwitch maxSwitch = LimitSwitch.createDIO(1, 
 *         () -> System.out.println("Max limit triggered"), null);
 * 
 * // Create the limited PID subsystem:
 * LimitedPIDSubsystem subsystem = new LimitedPIDSubsystem(
 *         10,            // CAN ID
 *         1.0,           // conversionFactor
 *         0.0,           // minPosition
 *         100.0,         // maxPosition
 *         new PID(0.1, 0.0, 0.0), // PID constants
 *         minSwitch,     // min limit switch
 *         maxSwitch,     // max limit switch
 *         5.0,           // tolerance
 *         LimitedPIDSubsystem.ControlMode.POSITION);
 * }
 * </pre>
 */
public class LimitedPIDSubsystem {
    /** Control mode for the subsystem. */
    public enum ControlMode {
        POSITION,
        VELOCITY
    }

    /** State of position knowledge for a subsystem. */
    public enum SubsystemState {
        KNOWN, UNKNOWN
    }

    /** The motor controller. */
    private final SparkMax motor;
    /** Current state of the subsystem. */
    private SubsystemState state = SubsystemState.UNKNOWN;
    /** Pair of limit switches for position limits. */
    private final LimitSwitchPair limitSwitches;
    /** Conversion factor from motor rotations to position units. */
    private final double conversionFactor;
    /** PID constants for position control. */
    private final PID pidConstants;
    /** Minimum allowed position value. */
    private final double minPosition;
    /** Maximum allowed position value. */
    private final double maxPosition;
    /** Tolerance for position offset. */
    private final double tolerance;
    /** Control mode for the subsystem. */
    private final ControlMode controlMode;

    /**
     * Creates a new motor subsystem with position/velocity control and pre-created limit switches.
     * This is the only supported constructor as it promotes cleaner code organization.
     *
     * @param canId The CAN ID of the motor controller
     * @param conversionFactor Factor to convert motor rotations to position/velocity units
     * @param minPosition Minimum allowed position value
     * @param maxPosition Maximum allowed position value
     * @param pidConstants PID constants for control
     * @param minLimitSwitch Pre-created limit switch for minimum position
     * @param maxLimitSwitch Pre-created limit switch for maximum position
     * @param tolerance Tolerance for position offset
     * @param controlMode Whether to use position or velocity control
     */
    public LimitedPIDSubsystem(int canId, double conversionFactor, double minPosition, double maxPosition,
            PID pidConstants, LimitSwitch minLimitSwitch, LimitSwitch maxLimitSwitch, 
            double tolerance, ControlMode controlMode) {
        this.conversionFactor = conversionFactor;
        this.pidConstants = pidConstants;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.tolerance = tolerance;
        this.controlMode = controlMode;

        motor = new SparkMax(canId, MotorType.kBrushless);
        configureMotor();

        // Configure limit switch callbacks
        minLimitSwitch = LimitSwitch.addCallback(minLimitSwitch, () -> handleMinLimit(), null);
        maxLimitSwitch = LimitSwitch.addCallback(maxLimitSwitch, () -> handleMaxLimit(), null);

        // Create limit switch pair using the switches
        limitSwitches = new LimitSwitchPair(
                minLimitSwitch::getBoolean,
                maxLimitSwitch::getBoolean,
                null,  // Callbacks already set on the switches
                null); // Callbacks already set on the switches

        initializePosition();
    }

    /**
     * Initializes the position of the mechanism based on limit switches.
     */
    private void initializePosition() {
        if (limitSwitches.isAtMin()) {
            setPositionAndTarget(minPosition);
        } else if (limitSwitches.isAtMax()) {
            setPositionAndTarget(maxPosition);
        } else {
            DriverStation.reportWarning("Position unknown for motor " + motor.getDeviceId(), false);
        }
    }

    /**
     * Configures the motor with the specified settings.
     */
    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(conversionFactor);
        config.apply(encoderConfig);

        ClosedLoopConfig pidConfig = new ClosedLoopConfig();
        pidConfig.pid(pidConstants.p, pidConstants.i, pidConstants.d);
        config.apply(pidConfig);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Sets the current position and target position of the motor.
     *
     * @param position The position to set
     */
    private void setPositionAndTarget(double position) {
        state = SubsystemState.KNOWN;
        motor.getEncoder().setPosition(position);
        if (controlMode == ControlMode.POSITION) {
            set(position);
        } else {
            set(0); // For velocity mode, stop at limits
        }
    }

    /**
     * Handles when minimum limit switch is triggered.
     */
    private void handleMinLimit() {
        motor.stopMotor();
        double currentPosition = motor.getEncoder().getPosition();
        double offset = Math.abs(minPosition - currentPosition);
        if (offset > tolerance) {
            DriverStation.reportError(
                String.format("Large position offset detected at min limit: %.2f units", offset),
                false
            );
        }
        motor.getEncoder().setPosition(minPosition);
        set(minPosition);
        state = SubsystemState.KNOWN;
    }

    /**
     * Handles when maximum limit switch is triggered.
     */
    private void handleMaxLimit() {
        motor.stopMotor();
        double currentPosition = motor.getEncoder().getPosition();
        double offset = Math.abs(maxPosition - currentPosition);
        if (offset > tolerance) {
            DriverStation.reportError(
                String.format("Large position offset detected at max limit: %.2f units", offset),
                false
            );
        }
        motor.getEncoder().setPosition(maxPosition);
        set(maxPosition);
        state = SubsystemState.KNOWN;
    }

    /**
     * Applies zeroing power if position is unknown.
     */
    public void zeroIfNeeded() {
        if (state == SubsystemState.UNKNOWN) {
            motor.set(CoralConstants.UNKNOWN_STATE_POWER);
        }
    }

    /**
     * Gets the current state of the subsystem.
     */
    public SubsystemState getState() {
        return state;
    }

    /**
     * Gets the current position of the mechanism.
     * @return current position in mechanism units
     */
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    /**
     * Sets the target value (position or velocity) of the mechanism.
     * For position control, value is clamped to valid range.
     * For velocity control, value is zeroed at limits.
     * @param value target value in mechanism units
     */
    public void set(double value) {
        if (controlMode == ControlMode.POSITION) {
            motor.getClosedLoopController().setReference(
                MathUtil.clamp(value, minPosition, maxPosition), 
                ControlType.kPosition
            );
        } else {
            // For velocity, zero at limits to prevent overrun
            if ((limitSwitches.isAtMin() && value < 0) || 
                (limitSwitches.isAtMax() && value > 0)) {
                motor.getClosedLoopController().setReference(0, ControlType.kVelocity);
            } else {
                motor.getClosedLoopController().setReference(value, ControlType.kVelocity);
            }
        }
    }

    /**
     * Gets the motor controller for this subsystem.
     * @return the SparkMax motor controller
     */
    public SparkMax getMotor() {
        return motor;
    }
}