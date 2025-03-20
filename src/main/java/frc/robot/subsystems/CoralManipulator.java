package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralManipulatorConstants;

/**
 * Subsystem that controls the coral manipulator (intake/output mechanism).
 * This handles the basic motor control and sensor detection for the coral game
 * pieces.
 */
public class CoralManipulator extends SubsystemBase {
    private static CoralManipulator instance;

    public static CoralManipulator getInstance() {
        if (instance == null) {
            instance = new CoralManipulator();
        }
        return instance;
    }

    private final SparkMax coralMotor = new SparkMax(CoralManipulatorConstants.CAN_ID, MotorType.kBrushless);
    private final DigitalInput coralSensor = new DigitalInput(CoralManipulatorConstants.SENSOR_CHANNEL);
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Robot")
            .getSubTable("CoralManipulator");

    private CoralManipulator() {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.inverted(CoralManipulatorConstants.INVERTED);
        sparkMaxConfig.smartCurrentLimit(30);
        coralMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        setDefaultCommand(stop());
    }

    public final Trigger running = new Trigger(
            () -> coralMotor.get() != 0);

    public final Trigger doesntHaveCoral = new Trigger(
            () -> coralSensor.get()).debounce(0.25);

    public final Trigger hasCoral = new Trigger(
            () -> !coralSensor.get());

    public Command intake() {
        return run(() -> coralMotor.set(CoralManipulatorConstants.INTAKE_POWER)).onlyWhile(doesntHaveCoral)
                .andThen(run(() -> {
                    coralMotor.set(0.0);
                }).withTimeout(0.5));
    }

    public Command drop() {
        return run(() -> coralMotor.set(CoralManipulatorConstants.DROP_POWER)).onlyWhile(hasCoral)
                .andThen(run(() -> {
                    coralMotor.set(0.0);
                }).withTimeout(0.5));
    }

    public Command eject() {
        return run(() -> coralMotor.set(CoralManipulatorConstants.EJECT_POWER));
    }

    public Command stop() {
        return run(() -> stopMotor());
    }

    public Command instantStop() {
        return runOnce(() -> stopMotor());
    }

    private void stopMotor() {
        coralMotor.set(0.0);
    }

    /**
     * Command that toggles between different states:
     * - If stopped: Drops coral if detected, otherwise ejects
     * - If already running: Stops all operations
     * 
     * This provides an easy way to perform the most intuitive action based on
     * context.
     * 
     * @return Command that performs the appropriate action based on current state
     */
    public Command ejectOrDrop() {
        return runOnce(() -> {
            if (running.getAsBoolean()) {
                stop().schedule();
            } else if (hasCoral.getAsBoolean()) {
                drop().schedule();
            } else {
                eject().schedule();
            }
        }).withName("ejectOrDrop");
    }

    @Override
    public void periodic() {
        table.getEntry("running").setBoolean(running.getAsBoolean());
        table.getEntry("hasCoral").setBoolean(hasCoral.getAsBoolean());
    }
}
