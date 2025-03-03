package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.AlgaArmConstants;
import frc.robot.subsubsytems.LimitSwitch;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Controls the alga arm mechanism for game piece manipulation.
 * This subsystem manages a single motor used for intaking and controlling alga
 * game pieces.
 * 
 * <p>
 * Features include:
 * <ul>
 * <li>Simple voltage-based control for intake
 * <li>Game piece detection
 * <li>Brake mode for secure holding
 * </ul>
 * 
 * <p>
 * Uses a REV Robotics SparkMax controller in brushless mode with brake mode
 * enabled
 * for precise control and secure game piece retention.
 */
public class AlgaArm extends SubsystemBase {
    /** Singleton instance of the AlgaArm subsystem. */
    private static AlgaArm instance;
    /** Motor controller for the alga intake mechanism. */
    private final SparkMax sparkMax;
    /** Limit switch for detecting alga game pieces. */
    private final LimitSwitch algaSensor;
    /** NetworkTable for publishing alga detection state. */
    private final NetworkTable table;

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
     * Initializes the motor controller with brake mode enabled for secure game
     * piece holding.
     * Motor configuration is non-persistent to ensure consistent behavior across
     * reboots.
     */
    public AlgaArm() {
        sparkMax = new SparkMax(AlgaArmConstants.CAN_ID, MotorType.kBrushless);
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        // Apply invert option from constants
        sparkMaxConfig.inverted(AlgaArmConstants.ALGA_INVERTED);
        sparkMax.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        algaSensor = new LimitSwitch(AlgaArmConstants.SENSOR_CHANNEL, () -> {
            if (sparkMax.get() > 0) {
                stopIntake();
            }
        }, () -> {
            if (sparkMax.get() < 0) {
                stopIntake();
            }
        });

        table = NetworkTableInstance.getDefault().getTable("AlgaArm");
    }

    /**
     * Activates the intake motor to collect alga game pieces.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * AlgaArm.getInstance().startIntake();
     * }
     * </pre>
     */
    public void startIntake() {
        sparkMax.set(AlgaArmConstants.INTAKE_POWER);
    }

    /**
     * Runs the motor in reverse to release the alga.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * AlgaArm.getInstance().dropGamepiece();
     * }
     * </pre>
     */
    public void dropGamepiece() {
        sparkMax.set(AlgaArmConstants.OUTTAKE_POWER);
    }

    /**
     * Stops the intake motor.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * AlgaArm.getInstance().stopIntake();
     * }
     * </pre>
     */
    public void stopIntake() {
        sparkMax.stopMotor();
    }

    /**
     * Checks if the alga game piece is detected.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * boolean hasPiece = AlgaArm.getInstance().hasGamepiece();
     * }
     * </pre>
     */
    public boolean hasGamepiece() {
        return algaSensor.get();
    }

    @Override
    public void periodic() {
        table.getEntry("hasAlga").setBoolean(hasGamepiece());
    }
}
