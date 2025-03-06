package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Climb extends SubsystemBase {
    /** Singleton instance of the AlgaArm subsystem. */
    private static Climb instance;

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    private final SparkMax climbMotor = new SparkMax(ClimbConstants.CAN_ID, MotorType.kBrushless);
    private final DigitalInput topLimit = new DigitalInput(ClimbConstants.TOP_LIMIT_CHANNEL);
    private final DigitalInput bottomLimit = new DigitalInput(ClimbConstants.BOTTOM_LIMIT_CHANNEL);
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Climb");

    private Climb() {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.inverted(ClimbConstants.IS_INVERTED);
        sparkMaxConfig.smartCurrentLimit(30);
        climbMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public final Trigger atTop = new Trigger(
            () -> !topLimit.get());

    public final Trigger atBottom = new Trigger(
            () -> !bottomLimit.get());

    public Command climb() {
        return run(() -> {
            climbMotor.set(ClimbConstants.CLIMB_POWER);
        }).onlyWhile(atBottom.negate()).finallyDo((interrupted) -> {
            stop().schedule();
        }).withName("climb");
    }

    public Command stop() {
        return runOnce(() -> {
            climbMotor.set(0);
        }).withName("stop");
    }

    @Override
    public void periodic() {
        table.getEntry("bottomLimit").setBoolean(atBottom.getAsBoolean());
        table.getEntry("topLimit").setBoolean(atTop.getAsBoolean());
    }
}
