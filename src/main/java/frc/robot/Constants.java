package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

        public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(5.45);
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
        public static final double CLIMB_POWER = -0.7;
        public static final boolean IS_INVERTED = false;
    }

    /**
     * Constants for operator interface (OI).
     */
    public static final class OIConstants {
        private OIConstants() {
        }

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DRIVER_DEADBAND = 0.002;
    }

    /**
     * Constants for the coral arm mechanism (elevator and elbow).
     */
    public static final class CoralArmConstants {
        private CoralArmConstants() {
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
        /** Minimum elevator height. */
        public static final Distance ELEVATOR_MIN_POSITION = Inch.of(0.0);
        /** Maximum elevator height. */
        public static final Distance ELEVATOR_MAX_POSITION = Inch.of(15.5);
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
        public static final int ELBOW_FRONT_LIMIT_CHANNEL = 4;
        public static final int ELBOW_BACK_LIMIT_CHANNEL = 3;
        public static final Angle ELBOW_FRONT_ANGLE = Degree.of(125.0);
        public static final Angle ELBOW_BACK_ANGLE = Degree.of(-125.0);
        public static final PID ELBOW_PID = new PID(0.01, 0.0, 0.0);
        /** Arm length. */
        public static final Distance ARM_LENGTH = Inch.of(30);
        public static final boolean ELBOW_INVERTED = false;
        public static final double ELBOW_ZEROING_POWER = 0.15;
        public static final double ELBOW_RAMP_RATE = 0.3;
        public static final int ELBOW_CURRENT_LIMIT = 20;

        // Shared constants
        /** Power applied when zeroing unknown positions (-1.0 to 1.0). */
        public static final double UNKNOWN_STATE_POWER = 0.1;
        /** Maximum allowed offset when hitting limit switches. */
        public static final double POSITION_TOLERANCE = 1.0;

        // Safe transition constants
        public static final Distance SAFE_ELEVATOR_HEIGHT = Inch.of(13.5);
        public static final double INTERMEDIATE_ELBOW_FRONT_ANGLE = 45.0; // degrees
        public static final double INTERMEDIATE_ELBOW_BACK_ANGLE = -45.0; // degrees
        
        /**
         * Represents a specific position state of the arm
         */
        public record ArmState(Angle elbowAngle, Distance elevatorHeight) {
            public boolean isFront() {
                return elbowAngle.in(Degree) > 0;
            }
        }
        
        /**
         * Predefined arm positions
         */
        public static final Map<String, ArmState> ARM_SETPOINTS = Map.of(
            "ZERO", new ArmState(ELBOW_FRONT_ANGLE, ELEVATOR_MIN_POSITION),
            "INTAKE", new ArmState(Degree.of(70), Inch.of(0.0)),
            "LOW", new ArmState(Degree.of(-90), Inch.of(5.0)),
            "MID", new ArmState(Degree.of(-35), Inch.of(0.0)),
            "HIGH", new ArmState(Degree.of(-35), Inch.of(15.0)),
            "CLIMB", new ArmState(Degree.of(-90), ELEVATOR_MAX_POSITION)
        );
    }

    /**
     * Constants for the coral manipulator (intake/output mechanism).
     */
    public static final class CoralManipulatorConstants {
        private CoralManipulatorConstants() {
        }

        /** CAN ID for manipulator motor */
        public static final int CAN_ID = 17;
        /** DIO channel for coral sensor */
        public static final int SENSOR_CHANNEL = 1;
        /** Power level for intake operation */
        public static final double INTAKE_POWER = 0.2;
        /** Power level for dropping coral */
        public static final double DROP_POWER = 1.0;
        /** Power level for ejecting stuck objects */
        public static final double EJECT_POWER = 1.0;
        /** Whether manipulator motor is inverted */
        public static final boolean INVERTED = false;
        /** Current limit for manipulator motor in amps */
        public static final int CURRENT_LIMIT = 20;
        /** Ramp rate for manipulator motor (seconds from 0 to full throttle) */
        public static final double RAMP_RATE = 0.1;
    }

    /**
     * Constants for pickup and coral side points.
     */
    public static final class PickupPoints {
        private PickupPoints() {
        }

        public static final Pose2d[] PICKUP_POINTS = new Pose2d[] {
                new Pose2d(1.0, 2.0, new Rotation2d(0)),
                new Pose2d(3.0, 4.0, new Rotation2d(0))
        };

        public static final Pose2d[] CORAL_SIDE_POINTS = new Pose2d[] {
                new Pose2d(5.0, 6.0, new Rotation2d(0)),
                new Pose2d(7.0, 8.0, new Rotation2d(0))
        };
    }

    public static final class ApriltagConstants {
        public static final class ApriltagCameraConfig {
            private String name;
            private Transform3d transform;
            private PoseStrategy strategy;

            public ApriltagCameraConfig(
                    String name,
                    Transform3d transform,
                    PoseStrategy strategy) {
                this.name = name;
                this.transform = transform;
                this.strategy = strategy;
            }

            public String getName() {
                return name;
            }

            public Transform3d getTransform() {
                return transform;
            }

            public PoseStrategy getStrategy() {
                return strategy;
            }
        }

        public static final ApriltagCameraConfig[] PHOTON_CAMERAS = {
                // new ApriltagCameraConfig(
                //         "Front Right",
                //         new Transform3d(
                //                 new Translation3d(
                //                         0.2794,
                //                         0.2794,
                //                         0.264),
                //                 new Rotation3d(
                //                         0,
                //                         0,
                //                         Degree.of(45).in(Radian))),
                //         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
                new ApriltagCameraConfig(
                        "Back Left",
                        new Transform3d(
                                new Translation3d(
                                        -0.2794,
                                        -0.2794,
                                        0.264),
                                new Rotation3d(
                                        0.0,
                                        0.0,
                                        Degree.of(135).in(Radian))),
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
                new ApriltagCameraConfig(
                        "Driver Cam",
                        new Transform3d(
                                new Translation3d(
                                        -0.075,
                                        -0.09,
                                        1.05),
                                new Rotation3d(
                                        0.0,
                                        Degree.of(-5).in(Radian),
                                        Degree.of(195).in(Radian))),
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
        };

        public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2025ReefscapeWelded
                .loadAprilTagLayoutField();

        /** Maximum allowed ambiguity for pose estimation (0-1, lower is better) */
        public static final double MAXIMUM_AMBIGUITY = 0.25;

        /** Standard deviations for single tag pose estimation */
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);

        /** Standard deviations for multi-tag pose estimation */
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

        /** Debounce time for camera reads in seconds */
        public static final double CAMERA_DEBOUNCE_TIME = 0.100; // Increased to 100ms to reduce processing frequency
        
        /** Maximum number of camera results to keep in memory */
        public static final int MAX_CAMERA_RESULTS = 1; // Only keep the latest result
    }

    /**
     * Constants for field positions and points of interest
     */
    public static final class FieldConstants {
        private FieldConstants() {
        }
        
        /** Field dimensions for 2025 Reefscape */
        public static final double FIELD_LENGTH_METERS = 16.54;
        public static final double FIELD_WIDTH_METERS = 8.21;
        
        /**
         * Point of Interest on the field
         * Contains a pose and an alliance designation
         */
        public record POI(Pose2d pose, Alliance alliance) {}
        
        /** Intake station locations */
        public static final POI[] INTAKE_STATIONS = {
            // Blue alliance intake station: x=1.25, y=7.0, rotation=126.0 degrees
            new POI(new Pose2d(1.25, 7.0, Rotation2d.fromDegrees(126.0)), Alliance.Blue),
            
            // Blue alliance mirrored across Y-axis (other side of the field)
            new POI(new Pose2d(1.25, FIELD_WIDTH_METERS - 7.0, Rotation2d.fromDegrees(-126.0)), Alliance.Blue),
            
            // Red alliance equivalent of first blue station (mirrored across field)
            new POI(new Pose2d(FIELD_LENGTH_METERS - 1.25, 7.0, Rotation2d.fromDegrees(126.0 + 180.0)), Alliance.Red),
            
            // Red alliance equivalent of second blue station (mirrored across field and Y-axis)
            new POI(new Pose2d(FIELD_LENGTH_METERS - 1.25, FIELD_WIDTH_METERS - 7.0, Rotation2d.fromDegrees(-126.0 + 180.0)), Alliance.Red)
        };
        
        /** Coral reef bar locations */
        public static final POI[] CORAL_REEF_BARS = {
            // Blue alliance reef bar 1: x=5.3, y=5.15, rotation=60 degrees
            new POI(new Pose2d(5.3, 5.15, Rotation2d.fromDegrees(60.0)), Alliance.Blue),
            
            // Blue alliance reef bar 2: x=5, y=5.3, rotation=60 degrees
            new POI(new Pose2d(5.0, 5.3, Rotation2d.fromDegrees(60.0)), Alliance.Blue),
            
            // Blue alliance reef bar 1 mirrored across Y-axis
            new POI(new Pose2d(5.3, FIELD_WIDTH_METERS - 5.15, Rotation2d.fromDegrees(-60.0)), Alliance.Blue),
            
            // Blue alliance reef bar 2 mirrored across Y-axis
            new POI(new Pose2d(5.0, FIELD_WIDTH_METERS - 5.3, Rotation2d.fromDegrees(-60.0)), Alliance.Blue),

            // Red alliance equivalent of blue reef bar 1 (mirrored across field)
            new POI(new Pose2d(FIELD_LENGTH_METERS - 5.3, 5.15, Rotation2d.fromDegrees(60.0 + 180.0)), Alliance.Red),
            
            // Red alliance equivalent of blue reef bar 2 (mirrored across field)
            new POI(new Pose2d(FIELD_LENGTH_METERS - 5.0, 5.3, Rotation2d.fromDegrees(60.0 + 180.0)), Alliance.Red),
            
            // Red alliance equivalent of blue reef bar 1 (mirrored across field and Y-axis)
            new POI(new Pose2d(FIELD_LENGTH_METERS - 5.3, FIELD_WIDTH_METERS - 5.15, Rotation2d.fromDegrees(-60.0 + 180.0)), Alliance.Red),
            
            // Red alliance equivalent of blue reef bar 2 (mirrored across field and Y-axis)
            new POI(new Pose2d(FIELD_LENGTH_METERS - 5.0, FIELD_WIDTH_METERS - 5.3, Rotation2d.fromDegrees(-60.0 + 180.0)), Alliance.Red)
        };
        
        /** Alga station locations */
        public static final POI[] ALGA_STATIONS = {
            // Blue alliance alga station: x=6.0, y=0.75, rotation=180 degrees
            new POI(new Pose2d(6.0, 0.75, Rotation2d.fromDegrees(180.0)), Alliance.Blue),
            
            // Red alliance equivalents (mirrored across field)
            new POI(new Pose2d(FIELD_LENGTH_METERS - 6.0, 0.75, Rotation2d.fromDegrees(0.0)), Alliance.Red),
        };
    }
}
