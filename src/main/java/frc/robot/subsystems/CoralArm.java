package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.CoralArmConstants;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArm extends SubsystemBase {
    private static CoralArm instance;
    private final SparkMax sparkMax;

    private CoralArmState state;

    private final AsynchronousInterrupt maxLimitSwitchInterrupt;
    private final AsynchronousInterrupt minLimitSwitchInterrupt;

    private enum CoralArmState {
        KNOWN,
        UNKNOWN
    }

    public static CoralArm getInstance() {
        if (instance == null) {
            instance = new CoralArm();
        }
        return instance;
    }
    
    public CoralArm() {
        sparkMax = new SparkMax(CoralArmConstants.ARM_CAN_ID, MotorType.kBrushless);
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(CoralArmConstants.ANGLE_PER_ROTATION.in(Degree));
        sparkMaxConfig.apply(encoderConfig);
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.pid(CoralArmConstants.CORALARM_MOTOR_PID.p, CoralArmConstants.CORALARM_MOTOR_PID.i, CoralArmConstants.CORALARM_MOTOR_PID.d);
        sparkMaxConfig.apply(closedLoopConfig);
        sparkMax.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        DigitalInput maxLimitSwitch = new DigitalInput(CoralArmConstants.MAX_LIMIT_SWITCH_CHANNEL);
        DigitalInput minLimitSwitch = new DigitalInput(CoralArmConstants.MIN_LIMIT_SWITCH_CHANNEL);

        state = CoralArmState.UNKNOWN;
        if (minLimitSwitch.get()) {
            atMin();
        } else if (maxLimitSwitch.get()) {
            atMax();
        } else {
            DriverStation.reportWarning("Elevator is in unknown position.", false);
        }

        maxLimitSwitchInterrupt = new AsynchronousInterrupt(maxLimitSwitch, this::maxLimitHit);
        minLimitSwitchInterrupt = new AsynchronousInterrupt(minLimitSwitch, this::minLimitHit);
        maxLimitSwitchInterrupt.enable();
        minLimitSwitchInterrupt.enable();
    }

    private void maxLimitHit(boolean rising, boolean falling) {
        if (rising) atMax();
    }

    private void minLimitHit(boolean rising, boolean falling) {
        if (rising) atMin();
    }

    private void atMax() {
        state = CoralArmState.KNOWN;
        sparkMax.getEncoder().setPosition(CoralArmConstants.MAX_ANGLE.in(Degree));
        sparkMax.getClosedLoopController().setReference(CoralArmConstants.MAX_ANGLE.in(Degree), ControlType.kPosition);
    }

    private void atMin() {
        state = CoralArmState.KNOWN;
        sparkMax.getEncoder().setPosition(0);
        sparkMax.getClosedLoopController().setReference(0, ControlType.kPosition);
    }

    public void zeroIfNeeded() {
        if (state == CoralArmState.UNKNOWN) {
            sparkMax.setVoltage(CoralArmConstants.UNKNOWN_STATE_VOLTAGE);
        }
    }
}
