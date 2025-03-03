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
 * Represents a PID-controlled subsystem with limit switches and position
 * control.
 */
public class LimitedPID {
    /** Control mode for the subsystem. */
    public enum ControlMode {
        POSITION,
        VELOCITY
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
    /** Ramp rate in seconds from 0 to full throttle */
    private final double rampRate;
    /** Maximum current limit in amps */
    private final int currentLimit;

    /** State of position knowledge for a subsystem. */
    public enum SubsystemState {
        KNOWN, UNKNOWN
    }

    /**
     * Creates a new motor subsystem with position/velocity control and limit switches.
     *
     * @param canId            CAN ID of the motor controller
     * @param conversionFactor Factor to convert motor rotations to position/velocity units
     * @param minPosition      Minimum allowed position value
     * @param maxPosition      Maximum allowed position value
     * @param pidConstants     PID constants for control
     * @param minLimitChannel  DIO channel for minimum position limit switch
     * @param maxLimitChannel  DIO channel for maximum position limit switch
     * @param tolerance        Tolerance for position offset
     * @param controlMode      Whether to use position or velocity control
     * @param isInverted       Whether the motor direction is inverted
     * @param rampRate         Ramp rate in seconds from 0 to full throttle
     * @param currentLimit     Maximum current limit in amps
     */
    public LimitedPID(int canId, double conversionFactor, double minPosition, double maxPosition,
            PID pidConstants, int minLimitChannel, int maxLimitChannel, double tolerance, ControlMode controlMode,
            boolean isInverted, double rampRate, int currentLimit) {
        this.conversionFactor = conversionFactor;
        this.pidConstants = pidConstants;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.tolerance = tolerance;
        this.controlMode = controlMode;
        this.isInverted = isInverted;
        this.rampRate = rampRate;
        this.currentLimit = currentLimit;

        motor = new SparkMax(canId, MotorType.kBrushless);
        configureMotor();

        limitSwitches = new LimitSwitchPair(
                minLimitChannel,
                maxLimitChannel,
                () -> handleMinLimit(),
                () -> handleMaxLimit());

        if (limitSwitches.isAtMin()) {
            setPositionAndTarget(minPosition);
        } else if (limitSwitches.isAtMax()) {
            setPositionAndTarget(maxPosition);
        } else {
            DriverStation.reportWarning("Position unknown for motor " + canId, false);
        }
    }

    /**
     * Configures the motor with the specified settings.
     */
    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(isInverted);
        config.openLoopRampRate(rampRate);
        config.closedLoopRampRate(rampRate);
        config.smartCurrentLimit(currentLimit);

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
        setPosition(minPosition);
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
        setPosition(maxPosition);
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
     * @deprecated Use set() instead
     */
    @Deprecated
    public void setPosition(double position) {
        if (controlMode != ControlMode.POSITION) {
            throw new UnsupportedOperationException("Subsystem is in velocity control mode");
        }
        set(position);
    }

    /**
     * Gets the motor controller for this subsystem.
     * @return the SparkMax motor controller
     */
    public SparkMax getMotor() {
        return motor;
    }
    
    /**
     * Checks if the mechanism is at the minimum limit.
     * @return true if at minimum limit, false otherwise
     */
    public boolean isAtMinLimit() {
        return limitSwitches.isAtMin();
    }
    
    /**
     * Checks if the mechanism is at the maximum limit.
     * @return true if at maximum limit, false otherwise
     */
    public boolean isAtMaxLimit() {
        return limitSwitches.isAtMax();
    }
}