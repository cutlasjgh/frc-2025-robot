package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

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
        public static final Voltage MOTOR_VOLTAGE = Volt.of(6.0);
    }

    /**
     * Constants for the climbing mechanism.
     */
    public static final class ClimbConstants {
        private ClimbConstants() {
        }

        public static final int CAN_ID = 0;
        public static final Voltage MOTOR_VOLTAGE = Volt.of(6.0);
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
        /** DIO channel for top limit switch. */
        public static final int ELEVATOR_TOP_LIMIT_CHANNEL = 0;
        /** DIO channel for bottom limit switch. */
        public static final int ELEVATOR_BOTTOM_LIMIT_CHANNEL = 1;
        /** Maximum elevator height. */
        public static final Distance ELEVATOR_HEIGHT = Inch.of(10.0);
        /** PID constants for elevator control. */
        public static final PID ELEVATOR_PID = new PID(0.0, 0.0, 0.0);

        // Arm constants
        /** CAN ID for arm motor. */
        public static final int ARM_CAN_ID = 1;
        /** Angle traveled per motor rotation. */
        public static final Angle ARM_ANGLE_PER_ROTATION = Degree.of(2.25);
        /** DIO channel for maximum angle limit switch. */
        public static final int ARM_MAX_LIMIT_CHANNEL = 2;
        /** DIO channel for minimum angle limit switch. */
        public static final int ARM_MIN_LIMIT_CHANNEL = 3;
        /** Maximum arm angle. */
        public static final Angle ARM_MAX_ANGLE = Degree.of(10.0);
        /** PID constants for arm control. */
        public static final PID ARM_PID = new PID(0.0, 0.0, 0.0);

        // Intake constants
        /** CAN ID for intake motor. */
        public static final int INTAKE_CAN_ID = 2;
        /** Operating voltage for intake motor. */
        public static final Voltage INTAKE_MOTOR_VOLTAGE = Volt.of(6.0);

        // Shared constants
        /** Voltage applied when zeroing unknown positions. */
        public static final Voltage UNKNOWN_STATE_VOLTAGE = Volt.of(6.0);
    }
}
