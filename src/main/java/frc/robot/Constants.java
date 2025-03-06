package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Constants used throughout the robot code.
 */
public final class Constants {
    /**
     * PID control constants container.
     */
    public static class PID {
        /** Proportional gain. */
        public final double p;
        /** Integral gain. */
        public final double i;
        /** Derivative gain. */
        public final double d;

        /**
         * Creates new PID constants.
         *
         * @param p Proportional gain
         * @param i Integral gain
         * @param d Derivative gain
         */
        private PID(double p, double i, double d) {
            this.p = p;
            this.i = i;
            this.d = d;
        }
    }

    private Constants() {
    }

    /**
     * Constants for basic robot characteristics.
     */
    public static final class RobotConstants {
        private RobotConstants() {
        }

        public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(15.0);
    }

    /**
     * Constants for the alga arm mechanism.
     */
    public static final class AlgaArmConstants {
        private AlgaArmConstants() {
        }

        public static final int CAN_ID = 19;
        public static final int SENSOR_CHANNEL = 7;
        public static final double INTAKE_POWER = 1.0;
        public static final double DROP_POWER = -1.0;
        public static final boolean ALGA_INVERTED = false;
    }

    /**
     * Constants for the climbing mechanism.
     */
    public static final class ClimbConstants {
        private ClimbConstants() {
        }

        public static final int CAN_ID = 14;
        public static final int BOTTOM_LIMIT_CHANNEL = 9;
        public static final int TOP_LIMIT_CHANNEL = 8;
        public static final double CLIMB_POWER = -0.05;
        public static final boolean IS_INVERTED = false;
    }

    /**
     * Constants for operator interface (OI).
     */
    public static final class OIConstants {
        private OIConstants() {
        }

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DRIVER_DEADBAND = 0.05;
    }

    /**
     * Constants for the coral mechanism (elevator, arm, and intake).
     */
    public static final class CoralSuperstructureConstants {
        private CoralSuperstructureConstants() {
        }

        // Elevator constants
        /** CAN ID for elevator motor. */
        public static final int ELEVATOR_CAN_ID = 15;
        /** Distance traveled per motor rotation. */
        public static final Distance ELEVATOR_DISTANCE_PER_ROTATION = Inch.of(0.5);
        /** DIO channel for elevator bottom limit switch */
        public static final int ELEVATOR_BOTTOM_LIMIT_CHANNEL = 0;
        /** CANifier pin for elevator top limit */
        public static final int ELEVATOR_TOP_LIMIT_CHANNEL = 2;
        /** Maximum elevator height. */
        public static final Distance ELEVATOR_HEIGHT = Inch.of(15.5);
        /** PID constants for elevator control. */
        public static final PID ELEVATOR_PID = new PID(0.1, 0.0, 0.0);
        /** Whether elevator motor is inverted */
        public static final boolean ELEVATOR_INVERTED = true;
        /** Power to use when zeroing elevator up */
        public static final double ELEVATOR_ZEROING_POWER_UP = 0.15;
        /** Power to use when zeroing elevator down */
        public static final double ELEVATOR_ZEROING_POWER_DOWN = -0.15;
        /** Ramp rate for elevator motor (seconds from 0 to full throttle) */
        public static final double ELEVATOR_RAMP_RATE = 0.25;
        /** Current limit for elevator motor in amps */
        public static final int ELEVATOR_CURRENT_LIMIT = 30;

        // Elbow constants
        public static final int ELBOW_CAN_ID = 16;
        /** Angle traveled per motor rotation. */
        public static final Angle ELBOW_ANGLE_PER_ROTATION = Degree.of(2.25);
        public static final int ELBOW_MAX_LIMIT_CHANNEL = 4;
        public static final int ELBOW_MIN_LIMIT_CHANNEL = 3;
        public static final Angle ELBOW_MAX_ANGLE = Degree.of(125.0);
        public static final Angle ELBOW_MIN_ANGLE = Degree.of(-125.0);
        public static final PID ELBOW_PID = new PID(0.01, 0.0, 0.0);
        /** Arm length. */
        public static final Distance ARM_LENGTH = Inch.of(30);
        public static final boolean ELBOW_INVERTED = false;
        public static final double ELBOW_ZEROING_POWER = 0.15;
        public static final double ELBOW_RAMP_RATE = 0.3;
        public static final int ELBOW_CURRENT_LIMIT = 20;

        // Intake constants
        public static final int MANIPULATOR_CAN_ID = 17;
        public static final int MANIPULATOR_SENSOR_CHANNEL = 1;
        public static final double INTAKE_POWER = 0.2;
        public static final double DROP_POWER = 1.0;
        public static final double EJECT_POWER = 1.0; // Power for ejecting stuck items
        public static final boolean MANIPULATOR_INVERTED = false;

        // Shared constants
        /** Power applied when zeroing unknown positions (-1.0 to 1.0). */
        public static final double UNKNOWN_STATE_POWER = 0.1;
        /** Maximum allowed offset when hitting limit switches. */
        public static final double POSITION_TOLERANCE = 1.0;

        // Add safe transition constants:
        public static final Distance SAFE_ELEVATOR_HEIGHT = Inch.of(13.5);
        public static final double INTERMEDIATE_ELBOW_FRONT_ANGLE = 45.0; // degrees
        public static final double INTERMEDIATE_ELBOW_BACK_ANGLE = -45.0; // degrees
    }

    /**
     * Constants for pickup and coral side points.
     */
    public static final class PickupPoints {
        private PickupPoints() {}
        public static final Pose2d[] PICKUP_POINTS = new Pose2d[]{
            new Pose2d(1.0, 2.0, new Rotation2d(0)),
            new Pose2d(3.0, 4.0, new Rotation2d(0))
        };

        public static final Pose2d[] CORAL_SIDE_POINTS = new Pose2d[]{
            new Pose2d(5.0, 6.0, new Rotation2d(0)),
            new Pose2d(7.0, 8.0, new Rotation2d(0))
        };
    }
}
