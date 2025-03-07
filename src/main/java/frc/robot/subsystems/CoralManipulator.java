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
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Robot").getSubTable("CoralManipulator");

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
        return run(() -> coralMotor.set(CoralManipulatorConstants.INTAKE_POWER)).onlyWhile(doesntHaveCoral);
    }

    public Command drop() {
        return run(() -> coralMotor.set(CoralManipulatorConstants.DROP_POWER)).onlyWhile(hasCoral);
    }

    public Command eject() {
        return run(() -> coralMotor.set(CoralManipulatorConstants.EJECT_POWER));
    }

    public Command stop() {
        return run(() -> {
            coralMotor.set(0.0);
        });
    }

    public Command toggle() {
        return runOnce(() -> {
            if (running.getAsBoolean()) {
                stop().schedule();
            } else {
                if (hasCoral.getAsBoolean()) {
                    drop().schedule();
                } else {
                    intake().schedule();
                }
            }
        }).withName("toggle");
    }

    @Override
    public void periodic() {
        table.getEntry("running").setBoolean(running.getAsBoolean());
        table.getEntry("hasCoral").setBoolean(hasCoral.getAsBoolean());
    }
}
