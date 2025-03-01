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
import frc.robot.Constants.PID;

/**
 * Represents a PID-controlled subsystem with limit switches and position
 * control.
 *
 * <p>
 * Example usage:
 * 
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
 *         10, // CAN ID
 *         1.0, // conversionFactor
 *         0.0, // minPosition
 *         100.0, // maxPosition
 *         new PID(0.1, 0.0, 0.0), // PID constants
 *         minSwitch, // min limit switch
 *         maxSwitch, // max limit switch
 *         5.0, // tolerance
 *         LimitedPIDSubsystem.ControlMode.POSITION);
 * }
 * </pre>
 */
public class LimitedPID {
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
    /** Whether the motor direction is inverted */
    private final boolean isInverted;
    /** Whether initialization is complete */
    private boolean initializationComplete = false;

    /**
     * Creates a new motor subsystem with position/velocity control, pre-created
     * limit switches,
     * and motor inversion setting.
     *
     * @param canId            The CAN ID of the motor controller
     * @param conversionFactor Factor to convert motor rotations to
     *                         position/velocity units
     * @param minPosition      Minimum allowed position value
     * @param maxPosition      Maximum allowed position value
     * @param pidConstants     PID constants for control
     * @param minLimitSwitch   Pre-created limit switch for minimum position
     * @param maxLimitSwitch   Pre-created limit switch for maximum position
     * @param tolerance        Tolerance for position offset
     * @param controlMode      Whether to use position or velocity control
     * @param isInverted       Whether to invert the motor direction
     */
    public LimitedPID(int canId, double conversionFactor, double minPosition, double maxPosition,
            PID pidConstants, LimitSwitch minLimitSwitch, LimitSwitch maxLimitSwitch,
            double tolerance, ControlMode controlMode, boolean isInverted) {
        this.conversionFactor = conversionFactor;
        this.pidConstants = pidConstants;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.tolerance = tolerance;
        this.controlMode = controlMode;
        this.isInverted = isInverted;

        motor = new SparkMax(canId, MotorType.kBrushless);
        configureMotor();

        // Configure limit switch callbacks with exit callbacks added
        minLimitSwitch = LimitSwitch.addCallback(minLimitSwitch, () -> handleMinLimit(), () -> exitMinLimit());
        maxLimitSwitch = LimitSwitch.addCallback(maxLimitSwitch, () -> handleMaxLimit(), () -> exitMaxLimit());

        // Create limit switch pair using the switches
        limitSwitches = new LimitSwitchPair(
                minLimitSwitch::getBoolean,
                maxLimitSwitch::getBoolean,
                null, // Callbacks already set on the switches
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
        initializationComplete = true;
    }

    /**
     * Configures the motor with the specified settings.
     */
    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(isInverted);

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
        if (!initializationComplete)
            return;

        motor.stopMotor();
        double currentPosition = motor.getEncoder().getPosition();
        double offset = Math.abs(minPosition - currentPosition);
        if (offset > tolerance) {
            DriverStation.reportError(
                    String.format("Large position offset detected at min limit: %.2f units", offset),
                    false);
        }
        motor.getEncoder().setPosition(minPosition);
        set(minPosition);
        state = SubsystemState.KNOWN;
    }

    /**
     * Handles when maximum limit switch is triggered.
     */
    private void handleMaxLimit() {
        if (!initializationComplete)
            return;
        motor.stopMotor();
        double currentPosition = motor.getEncoder().getPosition();
        double offset = Math.abs(maxPosition - currentPosition);
        if (offset > tolerance) {
            DriverStation.reportError(
                    String.format("Large position offset detected at max limit: %.2f units", offset),
                    false);
        }
        motor.getEncoder().setPosition(maxPosition);
        set(maxPosition);
        state = SubsystemState.KNOWN;
    }

    // New method to handle exit from min limit switch condition.
    private void exitMinLimit() {
        // Set control reference to the current position when exiting min limit.
        set(motor.getEncoder().getPosition());
    }

    // New method to handle exit from max limit switch condition.
    private void exitMaxLimit() {
        // Set control reference to the current position when exiting max limit.
        set(motor.getEncoder().getPosition());
    }

    /**
     * Returns the current position knowledge state of the subsystem.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * SubsystemState state = subsystem.getState();
     * }
     * </pre>
     */
    public SubsystemState getState() {
        return state;
    }

    /**
     * Returns the current position of the mechanism.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * double pos = subsystem.getPosition();
     * }
     * </pre>
     * 
     * @return current position in mechanism units
     */
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    /**
     * Sets the target (position or velocity) of the mechanism.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * subsystem.set(50.0);
     * }
     * </pre>
     * 
     * @param value target value in mechanism units
     */
    public void set(double value) {
        if (controlMode == ControlMode.POSITION) {
            motor.getClosedLoopController().setReference(
                    MathUtil.clamp(value, minPosition, maxPosition),
                    ControlType.kPosition);
        } else {
            if ((limitSwitches.isAtMin() && value < 0) ||
                    (limitSwitches.isAtMax() && value > 0)) {
                motor.getClosedLoopController().setReference(0, ControlType.kVelocity);
            } else {
                motor.getClosedLoopController().setReference(value, ControlType.kVelocity);
            }
        }
    }

    /**
     * Stops the motor.
     */
    public void stopMotor() {
        motor.stopMotor();
    }

    /**
     * Returns the underlying SparkMax motor controller.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * SparkMax motor = subsystem.getMotor();
     * }
     * </pre>
     * 
     * @return the motor controller
     */
    public SparkMax getMotor() {
        return motor;
    }

    /**
     * Returns whether the minimum limit switch is currently triggered.
     * 
     * @return true if at minimum limit, false otherwise
     */
    public boolean isAtMinLimit() {
        return limitSwitches.isAtMin();
    }

    /**
     * Returns whether the maximum limit switch is currently triggered.
     * 
     * @return true if at maximum limit, false otherwise
     */
    public boolean isAtMaxLimit() {
        return limitSwitches.isAtMax();
    }
}