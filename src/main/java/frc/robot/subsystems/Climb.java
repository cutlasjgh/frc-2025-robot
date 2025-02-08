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
 * Subsystem that handles the climbing mechanism.
 * Controls a motor with a ratchet for climbing operations.
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
     */
    private enum RachetState {
        /** Ratchet is engaged, preventing downward movement. */
        LOCKED,
        /** Ratchet is disengaged, allowing free movement. */
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
     * Initializes the motor controller and sets initial ratchet state.
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
     *
     * @return the current RachetState
     */
    public RachetState getState() {
        return state;
    }
}
