package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.AlgaArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the alga arm mechanism for game piece manipulation.
 * This subsystem manages a single motor used for intaking and controlling alga game pieces.
 * 
 * <p>Features include:
 * <ul>
 *   <li>Simple voltage-based control for intake
 *   <li>Game piece detection
 *   <li>Brake mode for secure holding
 * </ul>
 * 
 * <p>Uses a REV Robotics SparkMax controller in brushless mode with brake mode enabled
 * for precise control and secure game piece retention.
 */
public class AlgaArm extends SubsystemBase {
    /** Singleton instance of the AlgaArm subsystem. */
    private static AlgaArm instance;
    /** Motor controller for the alga intake mechanism. */
    private final SparkMax sparkMax;

    /**
     * Returns the singleton instance of the AlgaArm subsystem.
     * Creates a new instance if one does not exist.
     * 
     * @return the AlgaArm subsystem instance
     */
    public static AlgaArm getInstance() {
        if (instance == null) {
            instance = new AlgaArm();
        }
        return instance;
    }

    /**
     * Creates a new AlgaArm subsystem.
     * Initializes the motor controller with brake mode enabled for secure game piece holding.
     * Motor configuration is non-persistent to ensure consistent behavior across reboots.
     */
    public AlgaArm() {
        sparkMax = new SparkMax(AlgaArmConstants.CAN_ID, MotorType.kBrushless);
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMax.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Activates the intake motor to collect alga game pieces.
     * Applies a constant voltage defined in constants to ensure consistent intake behavior.
     * The motor will continue running until explicitly stopped.
     */
    public void intakeAlga() {
        sparkMax.setVoltage(AlgaArmConstants.MOTOR_VOLTAGE);
    }

    /**
     * Stops the intake motor's operation.
     * Due to brake mode, this will actively hold the game piece in place.
     */
    public void stopIntake() {
        sparkMax.setVoltage(0.0);
    }

    /**
     * Checks if the mechanism currently has an alga game piece.
     * Implementation pending for game piece detection.
     *
     * @return true if an alga game piece is detected, false otherwise
     */
    public boolean hasAlga() {
        return false;
    }
}
