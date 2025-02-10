package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the robot's climbing mechanism for end-game scoring.
 * This subsystem manages a motor with an integrated ratchet system that allows
 * controlled ascent while preventing uncontrolled descent.
 * 
 * <p>Features include:
 * <ul>
 *   <li>Ratchet-based climbing mechanism
 *   <li>State tracking for ratchet engagement
 *   <li>Safety features including brake mode
 * </ul>
 * 
 * <p>Uses a REV Robotics SparkMax controller with brake mode enabled
 * and includes a mechanical ratchet system for secure climbing operations.
 */
public class Climb extends SubsystemBase {
    /** Singleton instance of the Climb subsystem. */
    private static Climb instance;
    /** Motor controller for the climbing mechanism. */
    private final SparkMax sparkMax;
    /** Current state of the ratchet mechanism. */
    private final RachetState state;

    /**
     * Represents the possible states of the ratchet mechanism.
     * The ratchet provides one-way motion control for climbing operations.
     */
    private enum RachetState {
        /** Ratchet is engaged, preventing downward movement while allowing upward motion. */
        LOCKED,
        /** Ratchet is disengaged, allowing both upward and downward movement. */
        UNLOCKED
    }

    /**
     * Returns the singleton instance of the Climb subsystem.
     *
     * @return the Climb subsystem instance
     */
    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    /**
     * Creates a new Climb subsystem.
     * Initializes the motor controller with brake mode enabled and sets the initial
     * ratchet state to LOCKED for safety. Motor configuration is non-persistent
     * to ensure consistent behavior across reboots.
     */
    public Climb() {
        sparkMax = new SparkMax(ClimbConstants.CAN_ID, MotorType.kBrushless);
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMax.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        state = RachetState.LOCKED;
    }

    /**
     * Gets the current state of the ratchet mechanism.
     * Used to verify safe operation before movement commands.
     *
     * @return the current RachetState, indicating whether movement is restricted
     */
    public RachetState getState() {
        return state;
    }
}
