package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;

import java.util.ArrayList;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.PID;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * Controls the Coral mechanism, a multi-joint manipulator consisting of an elevator, arm, and intake.
 * This subsystem manages coordinated movement between components and provides preset positions
 * for common operations.
 * 
 * <p>Features include:
 * <ul>
 *   <li>Position-controlled elevator and arm using encoders and limit switches
 *   <li>Automatic position detection using limit switches
 *   <li>Predefined positions for common operations (ground pickup, scoring)
 *   <li>Safety features including position limits and state validation
 * </ul>
 * 
 * <p>The subsystem uses REV Robotics SparkMax controllers with closed-loop position control
 * and hardware limit switches for position detection and safety.
 */
public class CoralHandler extends SubsystemBase {
    private static CoralHandler instance;
    
    /**
     * Represents a PID-controlled subsystem with limit switches and position control.
     */
    private class LimitedPIDSubsystem {
        /** The motor controller. */
        private final SparkMax motor;
        /** Current state of the subsystem. */
        private SubsystemState state = SubsystemState.UNKNOWN;
        /** Pair of limit switches for position limits. */
        private final LimitSwitchPair limitSwitches;
        /** Conversion factor from motor rotations to position units. */
        private final double conversionFactor;
        /** PID constants for position control. */
        private final PID pidConstants;

        /**
         * Creates a new motor subsystem with position control and limit switches.
         *
         * @param canId CAN ID of the motor controller
         * @param conversionFactor Factor to convert motor rotations to position units
         * @param minPosition Minimum allowed position value
         * @param maxPosition Maximum allowed position value
         * @param pidConstants PID constants for position control
         * @param minLimitChannel DIO channel for minimum position limit switch
         * @param maxLimitChannel DIO channel for maximum position limit switch
         */
        public LimitedPIDSubsystem(int canId, double conversionFactor, double minPosition, double maxPosition, PID pidConstants,
                            int minLimitChannel, int maxLimitChannel) {
            this.conversionFactor = conversionFactor;
            this.pidConstants = pidConstants;

            motor = new SparkMax(canId, MotorType.kBrushless);
            configureMotor();

            limitSwitches = new LimitSwitchPair(
                minLimitChannel, 
                maxLimitChannel,
                () -> setPositionAndTarget(minPosition),
                () -> setPositionAndTarget(maxPosition)
            );

            if (limitSwitches.isAtMin()) {
                setPositionAndTarget(minPosition);
            } else if (limitSwitches.isAtMax()) {
                setPositionAndTarget(maxPosition);
            } else {
                DriverStation.reportWarning("Position unknown for motor " + canId, false);
            }
        }

        /**
         * Configures the motor with the specified settings.
         */
        private void configureMotor() {
            SparkMaxConfig config = new SparkMaxConfig();
            config.idleMode(IdleMode.kBrake);
            
            EncoderConfig encoderConfig = new EncoderConfig();
            encoderConfig.positionConversionFactor(conversionFactor);
            config.apply(encoderConfig);
            
            ClosedLoopConfig pidConfig = new ClosedLoopConfig();
            pidConfig.pid(pidConstants.p, pidConstants.i, pidConstants.d);
            config.apply(pidConfig);
            
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        /**
         * Sets the current position and target position of the motor.
         *
         * @param position The position to set
         */
        private void setPositionAndTarget(double position) {
            state = SubsystemState.KNOWN;
            motor.getEncoder().setPosition(position);
            motor.getClosedLoopController().setReference(position, ControlType.kPosition);
        }

        /**
         * Applies zeroing voltage if position is unknown.
         */
        public void zeroIfNeeded() {
            if (state == SubsystemState.UNKNOWN) {
                motor.setVoltage(CoralConstants.UNKNOWN_STATE_VOLTAGE);
            }
        }
    }

    /**
     * Manages a pair of limit switches with interrupt-based position detection.
     */
    private class LimitSwitchPair {
        private final AsynchronousInterrupt minInterrupt;
        private final AsynchronousInterrupt maxInterrupt;
        private final DigitalInput minSwitch;
        private final DigitalInput maxSwitch;

        /**
         * Creates a new limit switch pair with callbacks for position limits.
         * Uses interrupt-based detection for reliable position sensing.
         * 
         * <p>The callbacks are triggered on the rising edge of the limit switch signals,
         * indicating when a position limit is reached.
         *
         * @param minChannel DIO channel for minimum position switch
         * @param maxChannel DIO channel for maximum position switch
         * @param onMin Callback executed when minimum position is reached (rising edge)
         * @param onMax Callback executed when maximum position is reached (rising edge)
         */
        public LimitSwitchPair(int minChannel, int maxChannel, Runnable onMin, Runnable onMax) {
            minSwitch = new DigitalInput(minChannel);
            maxSwitch = new DigitalInput(maxChannel);

            minInterrupt = new AsynchronousInterrupt(minSwitch, (rising, falling) -> {
                if (rising) onMin.run();
            });
            maxInterrupt = new AsynchronousInterrupt(maxSwitch, (rising, falling) -> {
                if (rising) onMax.run();
            });

            minInterrupt.enable();
            maxInterrupt.enable();
        }

        /**
         * @return true if at minimum position
         */
        public boolean isAtMin() {
            return minSwitch.get();
        }

        /**
         * @return true if at maximum position
         */
        public boolean isAtMax() {
            return maxSwitch.get();
        }
    }

    /** State of position knowledge for a subsystem. */
    private enum SubsystemState { KNOWN, UNKNOWN }

    /**
     * Represents a configuration state of the arm mechanism.
     */
    private static class ArmState {
        public final Distance elevatorHeight;
        public final Angle armAngle;

        public ArmState(Distance elevatorHeight, Angle armAngle) {
            this.elevatorHeight = elevatorHeight;
            this.armAngle = armAngle;
        }

        /**
         * @return true if the state is within the physical limits of the mechanism
         */
        public boolean isValid() {
            return elevatorHeight.in(Inch) >= 0 && 
                   elevatorHeight.lte(CoralConstants.ELEVATOR_HEIGHT) &&
                   armAngle.gte(CoralConstants.ARM_MIN_ANGLE) &&
                   armAngle.lte(CoralConstants.ARM_MAX_ANGLE);
        }
    }

    /**
     * Predefined positions for the end effector.
     */
    private enum HandlerPosition {
        GROUND(new Translation2d(CoralConstants.ARM_LENGTH.in(Inch), 0)),
        SCORE(new Translation2d(0, CoralConstants.ELEVATOR_HEIGHT.in(Inch))),
        CUSTOM(null);

        public final Translation2d target;

        private HandlerPosition(Translation2d target) {
            this.target = target;
        }
    }

    /** Elevator subsystem instance. */
    private final LimitedPIDSubsystem elevator;
    /** Arm subsystem instance. */
    private final LimitedPIDSubsystem arm;
    /** Intake motor controller. */
    private final SparkMax intakeMotor;
    /** Current position setting. */
    private HandlerPosition currentPosition = HandlerPosition.CUSTOM;
    /** Arm length in inches. */
    private final double armLength;

    /**
     * @return Singleton instance of the CoralHandler
     */
    public static CoralHandler getInstance() {
        if (instance == null) {
            instance = new CoralHandler();
        }
        return instance;
    }
    
    /**
     * Creates a new CoralHandler, initializing all subsystems.
     */
    public CoralHandler() {
        elevator = new LimitedPIDSubsystem(
            CoralConstants.ELEVATOR_CAN_ID,
            CoralConstants.ELEVATOR_DISTANCE_PER_ROTATION.in(Inch),
            0,
            CoralConstants.ELEVATOR_HEIGHT.in(Inch),
            CoralConstants.ELEVATOR_PID,
            CoralConstants.ELEVATOR_BOTTOM_LIMIT_CHANNEL,
            CoralConstants.ELEVATOR_TOP_LIMIT_CHANNEL
        );

        arm = new LimitedPIDSubsystem(
            CoralConstants.ARM_CAN_ID,
            CoralConstants.ARM_ANGLE_PER_ROTATION.in(Degree),
            CoralConstants.ARM_MIN_ANGLE.in(Degree),
            CoralConstants.ARM_MAX_ANGLE.in(Degree),
            CoralConstants.ARM_PID,
            CoralConstants.ARM_MIN_LIMIT_CHANNEL,
            CoralConstants.ARM_MAX_LIMIT_CHANNEL
        );

        intakeMotor = new SparkMax(CoralConstants.INTAKE_CAN_ID, MotorType.kBrushless);
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        armLength = CoralConstants.ARM_LENGTH.in(Inch);
    }

    /**
     * Converts arm state to end effector position.
     */
    private Translation2d forwardKinematics(ArmState state) {
        double radians = Math.toRadians(state.armAngle.in(Degree));
        double x = armLength * Math.cos(radians);
        double y = state.elevatorHeight.in(Inch) + armLength * Math.sin(radians);
        return new Translation2d(x, y);
    }

    /**
     * Converts desired end effector position to arm state.
     * Returns null if position is unreachable.
     */
    private ArrayList<ArmState> inverseKinematics(Translation2d target) {
        ArrayList<Double> heightSolutions = new ArrayList<>();
        heightSolutions.add(target.getY() + Math.sqrt(Math.pow(armLength, 2) - Math.pow(target.getX(), 2)));
        heightSolutions.add(target.getY() - Math.sqrt(Math.pow(armLength, 2) - Math.pow(target.getX(), 2)));
        heightSolutions.removeIf(height -> height < 0 || height > CoralConstants.ELEVATOR_HEIGHT.in(Inch) || Double.isNaN(height));

        ArrayList<ArmState> validSolutions = new ArrayList<>();
        for (double height : heightSolutions) {
            double armAngle = Math.toDegrees(Math.atan2(target.getX(), target.getY() - height));
            if (armAngle >= CoralConstants.ARM_MIN_ANGLE.in(Degree) && armAngle <= CoralConstants.ARM_MAX_ANGLE.in(Degree)) {
                validSolutions.add(new ArmState(Inch.of(height), Degree.of(armAngle)));
            }
        }

        return validSolutions;
    }

    /**
     * Sets both elevator and arm to a predefined position.
     */
    public void setPosition(HandlerPosition targetPosition) {
        if (currentPosition == targetPosition || 
            elevator.state == SubsystemState.UNKNOWN || 
            arm.state == SubsystemState.UNKNOWN) return;

        if (targetPosition != HandlerPosition.CUSTOM && targetPosition.target != null) {
            ArrayList<ArmState> solutions = inverseKinematics(targetPosition.target);
            if (!solutions.isEmpty()) {
                ArmState solution = solutions.get(0); // Use first solution. This will be improved in the future.
                elevator.motor.getClosedLoopController().setReference(
                    solution.elevatorHeight.in(Inch), ControlType.kPosition);
                arm.motor.getClosedLoopController().setReference(
                    solution.armAngle.in(Degree), ControlType.kPosition);
                currentPosition = targetPosition;
            }
        }
    }

    /**
     * Sets the end effector to a custom 2D position.
     */
    public void setCustomPosition(Translation2d target) {
        ArrayList<ArmState> solutions = inverseKinematics(target);
        if (!solutions.isEmpty()) {
            ArmState solution = solutions.get(0); // Use first solution. This will be improved in the future.
            elevator.motor.getClosedLoopController().setReference(
                solution.elevatorHeight.in(Inch), ControlType.kPosition);
            arm.motor.getClosedLoopController().setReference(
                solution.armAngle.in(Degree), ControlType.kPosition);
            currentPosition = HandlerPosition.CUSTOM;
        }
    }

    /**
     * Applies zeroing voltage to subsystems with unknown positions.
     * This helps bring mechanisms to their limit switches for position detection.
     * 
     * <p>Only applies voltage if a subsystem's position is currently unknown.
     * Once a limit switch is triggered, the position will be updated automatically.
     */
    public void zeroIfNeeded() {
        elevator.zeroIfNeeded();
        arm.zeroIfNeeded();
    }
}
