package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inch;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private static Elevator instance;
    private final SparkMax sparkMax;

    private ElevatorState state;
    private ElevatorLevel level;

    private final AsynchronousInterrupt topLimitSwitchInterrupt;
    private final AsynchronousInterrupt bottomLimitSwitchInterrupt;

    private enum ElevatorState {
        KNOWN,
        UNKNOWN
    }

    private enum ElevatorLevel {
        BOTTOM(Inch.of(0)),
        TOP(ElevatorConstants.ELEVATOR_HEIGHT),
        CUSTOM(null);

        public final Distance target;

        private ElevatorLevel(Distance target) {
            this.target = target;
        }
    }

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }
    
    public Elevator() {
        sparkMax = new SparkMax(ElevatorConstants.CAN_ID, MotorType.kBrushless);
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(ElevatorConstants.DISTANCE_PER_ROTATION.in(Inch));
        sparkMaxConfig.apply(encoderConfig);
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.pid(ElevatorConstants.ELEVATOR_MOTOR_PID.p, ElevatorConstants.ELEVATOR_MOTOR_PID.i, ElevatorConstants.ELEVATOR_MOTOR_PID.d);
        sparkMaxConfig.apply(closedLoopConfig);
        sparkMax.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.TOP_LIMIT_SWITCH_CHANNEL);
        DigitalInput bottomLimitSwitch = new DigitalInput(ElevatorConstants.BOTTOM_LIMIT_SWITCH_CHANNEL);

        level = ElevatorLevel.CUSTOM;
        state = ElevatorState.UNKNOWN;
        if (bottomLimitSwitch.get()) {
            atBottom();
        } else if (topLimitSwitch.get()) {
            atTop();
        } else {
            DriverStation.reportWarning("Elevator is in unknown position.", false);
        }

        topLimitSwitchInterrupt = new AsynchronousInterrupt(topLimitSwitch, this::topLimitHit);
        bottomLimitSwitchInterrupt = new AsynchronousInterrupt(bottomLimitSwitch, this::bottomLimitHit);
        topLimitSwitchInterrupt.enable();
        bottomLimitSwitchInterrupt.enable();
    }

    private void topLimitHit(boolean rising, boolean falling) {
        if (rising) atTop();
    }

    private void bottomLimitHit(boolean rising, boolean falling) {
        if (rising) atBottom();
    }

    private void atTop() {
        state = ElevatorState.KNOWN;
        sparkMax.getEncoder().setPosition(ElevatorConstants.ELEVATOR_HEIGHT.in(Inch));
        setLevel(ElevatorLevel.TOP);
    }

    private void atBottom() {
        state = ElevatorState.KNOWN;
        sparkMax.getEncoder().setPosition(0);
        setLevel(ElevatorLevel.BOTTOM);
    }

    public void zeroIfNeeded() {
        if (state == ElevatorState.UNKNOWN) {
            sparkMax.setVoltage(ElevatorConstants.UNKNOWN_STATE_VOLTAGE);
        }
    }

    public void setLevel(ElevatorLevel targetLevel) {
        if (level == targetLevel || state == ElevatorState.UNKNOWN) return;

        if (targetLevel != ElevatorLevel.CUSTOM)
            sparkMax.getClosedLoopController().setReference(targetLevel.target.in(Inch), ControlType.kPosition);
        level = targetLevel;
    }
}
