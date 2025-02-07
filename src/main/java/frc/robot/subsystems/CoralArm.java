package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.CoralArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArm extends SubsystemBase {
    private static CoralArm instance;
    private final SparkMax sparkMax;

    public static CoralArm getInstance() {
        if (instance == null) {
            instance = new CoralArm();
        }
        return instance;
    }

    public CoralArm() {
        sparkMax = new SparkMax(CoralArmConstants.CAN_ID, MotorType.kBrushless);
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMax.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void intakeCoral() {
        sparkMax.setVoltage(CoralArmConstants.MOTOR_VOLTAGE);
    }

    public void stopIntake() {
        sparkMax.setVoltage(0.0);
    }

    public boolean hasCoral() {
        return false;
    }
}
