package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
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

        public static final int CAN_ID = 0;
        public static final int SENSOR_CHANNEL = 7;
        public static final double INTAKE_POWER = 0.5;
        public static final double OUTTAKE_POWER = -0.5;
    }

    /**
     * Constants for the climbing mechanism.
     */
    public static final class ClimbConstants {
        private ClimbConstants() {
        }

        public static final int CAN_ID = 0;
        public static final Voltage MOTOR_VOLTAGE = Volt.of(6.0);
        public static final int BOTTOM_LIMIT_CHANNEL = 4;
        public static final int TOP_LIMIT_CHANNEL = 5;
        public static final double CLIMB_POWER = 0.5;
        public static final boolean IS_INVERTED = true;
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
    public static final class CoralConstants {
        private CoralConstants() {
        }

        // Elevator constants
        /** CAN ID for elevator motor. */
        public static final int ELEVATOR_CAN_ID = 0;
        /** Distance traveled per motor rotation. */
        public static final Distance ELEVATOR_DISTANCE_PER_ROTATION = Inch.of(0.5);
        /** DIO channel for elevator bottom limit switch */
        public static final int ELEVATOR_BOTTOM_LIMIT_CHANNEL = 0;
        /** CANifier pin for elevator top limit */
        public static final CANifier.GeneralPin ELEVATOR_TOP_LIMIT_PIN = CANifier.GeneralPin.SCL;
        /** Maximum elevator height. */
        public static final Distance ELEVATOR_HEIGHT = Inch.of(10.0);
        /** PID constants for elevator control. */
        public static final PID ELEVATOR_PID = new PID(0.0, 0.0, 0.0);

        // Arm constants
        /** CAN ID for arm motor. */
        public static final int ARM_CAN_ID = 1;
        /** Angle traveled per motor rotation. */
        public static final Angle ARM_ANGLE_PER_ROTATION = Degree.of(2.25);
        /** CANifier pin for arm maximum angle limit */
        public static final CANifier.GeneralPin ARM_MAX_LIMIT_PIN = CANifier.GeneralPin.LIMF;
        /** CANifier pin for arm minimum angle limit */
        public static final CANifier.GeneralPin ARM_MIN_LIMIT_PIN = CANifier.GeneralPin.LIMR;
        /** Maximum arm angle. */
        public static final Angle ARM_MAX_ANGLE = Degree.of(150.0);
        /** Minimum arm angle. */
        public static final Angle ARM_MIN_ANGLE = Degree.of(-150.0);
        /** PID constants for arm control. */
        public static final PID ARM_PID = new PID(0.0, 0.0, 0.0);
        /** Arm length. */
        public static final Distance ARM_LENGTH = Inch.of(30);

        // Intake constants
        /** CAN ID for intake motor. */
        public static final int INTAKE_CAN_ID = 2;
        /** CANifier pin for coral detection */
        public static final CANifier.GeneralPin INTAKE_SENSOR_PIN = CANifier.GeneralPin.SPI_CS;
        /** Intake motor power (-1.0 to 1.0). */
        public static final double INTAKE_POWER = 0.5;
        /** Outtake motor power (-1.0 to 1.0). */
        public static final double OUTTAKE_POWER = -0.5;

        // Shared constants
        /** CANifier ID */
        public static final int CANIFIER_ID = 0;
        /** Power applied when zeroing unknown positions (-1.0 to 1.0). */
        public static final double UNKNOWN_STATE_POWER = 0.1;
        /** Maximum allowed offset when hitting limit switches. */
        public static final double POSITION_TOLERANCE = 1.0;
    }

    /**
     * Constants for pickup and coral side points.
     */
    public static final class PickupPoints {
        private PickupPoints() {}
        public static final Pose2d[] PICKUP_POINTS = new Pose2d[]{
            new Pose2d(1.0, 2.0, new Rotation2d(0)),
            new Pose2d(3.0, 4.0, new Rotation2d(0))
            // ...add more as needed...
        };

        public static final Pose2d[] CORAL_SIDE_POINTS = new Pose2d[]{
            new Pose2d(5.0, 6.0, new Rotation2d(0)),
            new Pose2d(7.0, 8.0, new Rotation2d(0))
            // ...add more as needed...
        };
    }
}
