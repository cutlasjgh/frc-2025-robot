package frc.robot.helpers;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.PID;

/**
 * Represents a PID-controlled subsystem with limit switches and position control.
 */
public class LimitedPID {
    /** The motor controller. */
    private final SparkMax motor;
    /** Current state of the subsystem. */
    private SubsystemState state = SubsystemState.UNKNOWN;
    /** Minimum limit switch. */
    private final DigitalInput minLimitSwitch;
    /** Maximum limit switch. */
    private final DigitalInput maxLimitSwitch;
    /** Previous state of min limit switch (true = pressed) */
    private boolean prevMinLimitPressed = false;
    /** Previous state of max limit switch (true = pressed) */
    private boolean prevMaxLimitPressed = false;
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
     * Creates a new motor subsystem with position control and limit switches.
     *
     * @param canId            CAN ID of the motor controller
     * @param conversionFactor Factor to convert motor rotations to position units
     * @param minPosition      Minimum allowed position value
     * @param maxPosition      Maximum allowed position value
     * @param pidConstants     PID constants for control
     * @param minLimitChannel  DIO channel for minimum position limit switch
     * @param maxLimitChannel  DIO channel for maximum position limit switch
     * @param tolerance        Tolerance for position offset
     * @param isInverted       Whether the motor direction is inverted
     * @param rampRate         Ramp rate in seconds from 0 to full throttle
     * @param currentLimit     Maximum current limit in amps
     */
    public LimitedPID(int canId, double conversionFactor, double minPosition, double maxPosition,
            PID pidConstants, int minLimitChannel, int maxLimitChannel, double tolerance,
            boolean isInverted, double rampRate, int currentLimit) {
        this.conversionFactor = conversionFactor;
        this.pidConstants = pidConstants;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.tolerance = tolerance;
        this.isInverted = isInverted;
        this.rampRate = rampRate;
        this.currentLimit = currentLimit;

        motor = new SparkMax(canId, MotorType.kBrushless);
        configureMotor();

        minLimitSwitch = new DigitalInput(minLimitChannel);
        maxLimitSwitch = new DigitalInput(maxLimitChannel);
        
        // Initialize limit switch states
        prevMinLimitPressed = isAtMinLimit();
        prevMaxLimitPressed = isAtMaxLimit();
        
        // Initialize position if at a limit
        if (isAtMinLimit()) {
            setPositionAndTarget(minPosition);
        } else if (isAtMaxLimit()) {
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
        setPosition(position);
    }

    /**
     * Handles when minimum limit switch is triggered.
     */
    private void handleMinLimit() {
        double currentPosition = motor.getEncoder().getPosition();
        double positionError = Math.abs(currentPosition - minPosition);
        
        // Only take action if position is significantly different from expected
        if (positionError > tolerance) {
            motor.getEncoder().setPosition(minPosition);
        }
        
        state = SubsystemState.KNOWN;
    }

    /**
     * Handles when maximum limit switch is triggered.
     */
    private void handleMaxLimit() {
        double currentPosition = motor.getEncoder().getPosition();
        double positionError = Math.abs(currentPosition - maxPosition);
        
        // Only take action if position is significantly different from expected
        if (positionError > tolerance) {
            motor.getEncoder().setPosition(maxPosition);
        }
        
        state = SubsystemState.KNOWN;
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
     * Sets the target position of the mechanism.
     * Position is clamped to valid range.
     * @param position target position in mechanism units
     */
    public void setPosition(double position) {
        motor.getClosedLoopController().setReference(
            MathUtil.clamp(position, minPosition, maxPosition), 
            ControlType.kPosition
        );
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
        return !minLimitSwitch.get();
    }
    
    /**
     * Checks if the mechanism is at the maximum limit.
     * @return true if at maximum limit, false otherwise
     */
    public boolean isAtMaxLimit() {
        return !maxLimitSwitch.get();
    }
    
    /**
     * Updates the mechanism status by checking limit switches.
     */
    public void update() {
        if (isAtMinLimit()) {
            handleMinLimit();
        }
        
        if (isAtMaxLimit()) {
            handleMaxLimit();
        }
    }
}