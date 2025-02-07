package frc.robot;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public final class Constants {
    public static class PID {
        public final double p;
        public final double i;
        public final double d;

        private PID(double p, double i, double d) {
            this.p = p;
            this.i = i;
            this.d = d;
        }
    }


    private Constants() {
    }

    public static final class RobotConstants {
        private RobotConstants() {
        }

        public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(15.0);
    }

    public static final class AlgaArmConstants {
        private AlgaArmConstants() {
        }

        public static final int CAN_ID = 0;
        public static final Voltage MOTOR_VOLTAGE = Volt.of(6.0);
    }

    public static final class ClimbConstants {
        private ClimbConstants() {
        }

        public static final int CAN_ID = 0;
        public static final Voltage MOTOR_VOLTAGE = Volt.of(6.0);
    }

    public static final class OIConstants {
        private OIConstants() {
        }

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DRIVER_DEADBAND = 0.05;
    }

    public static final class CoralArmConstants {
        private CoralArmConstants() {
        }

        public static final int CAN_ID = 0;
        public static final Voltage MOTOR_VOLTAGE = Volt.of(6.0);
    }

    public static final class ElevatorConstants {
        private ElevatorConstants() {
        }

        public static final int CAN_ID = 0;
        public static final Distance DISTANCE_PER_ROTATION = Inch.of(0.5);
        public static final int TOP_LIMIT_SWITCH_CHANNEL = 0;
        public static final int BOTTOM_LIMIT_SWITCH_CHANNEL = 1;
        public static final Distance ELEVATOR_HEIGHT = Inch.of(10.0);
        public static final PID ELEVATOR_MOTOR_PID = new PID(0.0, 0.0, 0.0);
        public static final Voltage UNKNOWN_STATE_VOLTAGE = Volt.of(6.0);
    }
}
