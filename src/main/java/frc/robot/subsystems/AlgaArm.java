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
import frc.robot.Constants.AlgaArmConstants;

// Make this into commands that keep running until they are canceled or stop instead of a blanket activation

public class AlgaArm extends SubsystemBase {
    /** Singleton instance of the AlgaArm subsystem. */
    private static AlgaArm instance;

    public static AlgaArm getInstance() {
        if (instance == null) {
            instance = new AlgaArm();
        }
        return instance;
    }

    /** Motor controller for the alga intake mechanism. */
    private final SparkMax algaMotor = new SparkMax(AlgaArmConstants.CAN_ID, MotorType.kBrushless);
    /** Limit switch for detecting alga game pieces. */
    private final DigitalInput algaSensor = new DigitalInput(AlgaArmConstants.SENSOR_CHANNEL);
    /** NetworkTable for publishing alga detection state. */
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("AlgaArm");

    private AlgaArm() {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.inverted(AlgaArmConstants.ALGA_INVERTED);
        sparkMaxConfig.smartCurrentLimit(20);
        sparkMaxConfig.openLoopRampRate(0.1);
        sparkMaxConfig.closedLoopRampRate(0.1);
        algaMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public final Trigger doesntHaveAlga = new Trigger(
            () -> !algaSensor.get()).debounce(0.25);

    public final Trigger hasAlga = new Trigger(
            () -> algaSensor.get());

    public final Trigger running = new Trigger(
            () -> {
                return algaMotor.get() != 0;
            });

    public Command intake() {
        return run(() -> {
            algaMotor.set(AlgaArmConstants.INTAKE_POWER);
        }).onlyWhile(doesntHaveAlga).finallyDo((interrupted) -> {
            stop().schedule();
        }).withName("algaIntake");
    }

    public Command drop() {
        return run(() -> {
            algaMotor.set(AlgaArmConstants.DROP_POWER);
        }).onlyWhile(hasAlga).finallyDo((interrupted) -> {
            stop().schedule();
        }).withName("algaDrop");
    }

    public Command stop() {
        return run(() -> {
            algaMotor.set(0);
        }).withName("algaStop");
    }

    public Command toggle() {
        return runOnce(() -> {
            if (running.getAsBoolean()) {
                stop().schedule();
            } else {
                if (hasAlga.getAsBoolean()) {
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
        table.getEntry("hasAlga").setBoolean(hasAlga.getAsBoolean());
    }
}
