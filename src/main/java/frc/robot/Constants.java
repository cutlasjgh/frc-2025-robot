package frc.robot;

public final class Constants {
    private Constants() {
    }

    public static final class RobotConstants {
        private RobotConstants() {
        }

        public static final double MAX_SPEED = 15.0;
    }

    public static final class AlgaArmConstants {
        private AlgaArmConstants() {
        }

        public static final int CAN_ID = 0;
        public static final double MOTOR_VOLTAGE = 6.0;
    }

    public static final class ClimbConstants {
        private ClimbConstants() {
        }

        public static final int CAN_ID = 0;
        public static final double MOTOR_VOLTAGE = 6.0;
        public static final double INCHES_PER_ROTATION = 0.5;
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
        public static final double MOTOR_VOLTAGE = 6.0;
    }

    public static final class ElevatorConstants {
        private ElevatorConstants() {
        }

        public static final int CAN_ID = 0;
        public static final double MOTOR_VOLTAGE = 6.0;
    }
}
