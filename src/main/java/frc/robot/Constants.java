package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Arrays;
import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.util.FlippingUtil;

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
        /**
         * Private constructor to prevent instantiation.
         */
        private AlgaArmConstants() {
        }

        /**
         * CAN ID for the alga motor.
         */
        public static final int CAN_ID = 19;
        /**
         * Sensor channel for the alga sensor.
         */
        public static final int SENSOR_CHANNEL = 7;
        /**
         * Intake power for the alga arm.
         */
        public static final double INTAKE_POWER = 1.0;
        /**
         * Drop power for the alga arm.
         */
        public static final double DROP_POWER = -1.0;
        /**
         * Whether the alga arm is inverted.
         */
        public static final boolean ALGA_INVERTED = false;
    }

    /**
     * Constants for the climbing mechanism.
     */
    public static final class ClimbConstants {
        /**
         * Private constructor to prevent instantiation.
         */
        private ClimbConstants() {
        }

        /**
         * CAN ID for the climb motor.
         */
        public static final int CAN_ID = 14;
        /**
         * Bottom limit channel for the climb.
         */
        public static final int BOTTOM_LIMIT_CHANNEL = 9;
        /**
         * Top limit channel for the climb.
         */
        public static final int TOP_LIMIT_CHANNEL = 8;
        /**
         * Climb power for the climb.
         */
        public static final double CLIMB_POWER = -0.7;
        /**
         * Whether the climb is inverted.
         */
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
                "INTAKE", new ArmState(Degree.of(70), Inch.of(1.25)),
                "LOW", new ArmState(Degree.of(-90), Inch.of(5.0)),
                "MID", new ArmState(Degree.of(-30), Inch.of(0.0)),
                "HIGH", new ArmState(Degree.of(-30), Inch.of(15.0)));
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
        public static final double EJECT_POWER = 0.5;
        /** Whether manipulator motor is inverted */
        public static final boolean INVERTED = false;
        /** Current limit for manipulator motor in amps */
        public static final int CURRENT_LIMIT = 20;
        /** Ramp rate for manipulator motor (seconds from 0 to full throttle) */
        public static final double RAMP_RATE = 0.1;
    }

    /**
     * Constants for apriltag vision.
     */
    public static final class ApriltagConstants {
        /**
         * Apriltag camera configuration.
         */
        public static final class ApriltagCameraConfig {
            /**
             * Name of the camera.
             */
            private String name;
            /**
             * Transform of the camera.
             */
            private Transform3d transform;
            /**
             * Strategy of the camera.
             */
            private PoseStrategy strategy;

            /**
             * Apriltag camera configuration.
             *
             * @param name      Name of the camera
             * @param transform Transform of the camera
             * @param strategy  Strategy of the camera
             */
            public ApriltagCameraConfig(
                    String name,
                    Transform3d transform,
                    PoseStrategy strategy) {
                this.name = name;
                this.transform = transform;
                this.strategy = strategy;
            }

            /**
             * Gets the name of the camera.
             *
             * @return Name of the camera
             */
            public String getName() {
                return name;
            }

            /**
             * Gets the transform of the camera.
             *
             * @return Transform of the camera
             */
            public Transform3d getTransform() {
                return transform;
            }

            /**
             * Gets the strategy of the camera.
             *
             * @return Strategy of the camera
             */
            public PoseStrategy getStrategy() {
                return strategy;
            }
        }

        /**
         * Photon cameras.
         */
        public static final ApriltagCameraConfig[] PHOTON_CAMERAS = {
                new ApriltagCameraConfig(
                        "Front Left",
                        new Transform3d(
                                new Translation3d(
                                        0.27305,
                                        0.28448,
                                        0.2921),
                                new Rotation3d(
                                        0,
                                        Degree.of(-20.0).in(Radian),
                                        Degree.of(-16.38).in(Radian))),
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
                new ApriltagCameraConfig(
                        "Back Right",
                        new Transform3d(
                                new Translation3d(
                                        -0.2794,
                                        -0.3175,
                                        0.2921),
                                new Rotation3d(
                                        0,
                                        Degree.of(-20.0).in(Radian),
                                        Degree.of(-196.38).in(Radian))),
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
                new ApriltagCameraConfig(
                        "Back Left",
                        new Transform3d(
                                new Translation3d(
                                        -0.28194,
                                        0.3048,
                                        0.2667),
                                new Rotation3d(
                                        0.0,
                                        Degree.of(-10.0).in(Radian),
                                        Degree.of(196.38).in(Radian))),
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
        };

        /**
         * Field layout for apriltags.
         */
        public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        /** Maximum allowed ambiguity for pose estimation (0-1, lower is better) */
        public static final double MAXIMUM_AMBIGUITY = 0.25;

        /** Standard deviations for single tag pose estimation */
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);

        /** Standard deviations for multi-tag pose estimation */
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

        /** Debounce time for camera reads in seconds */
        public static final double CAMERA_DEBOUNCE_TIME = 0.150;

        /** BULLSHIT */
        public static final double SINGLE_TAG_CUTOFF_METER = 4;

        /** Maximum number of camera results to keep in memory */
        public static final int MAX_CAMERA_RESULTS = 5;
    }

    /**
     * Constants for field positions and points of interest
     */
    public static final class FieldConstants {
        private FieldConstants() {
        }

        /** Field dimensions for 2025 Reefscape */
        public static final double FIELD_LENGTH_METERS = 17.548;
        public static final double FIELD_WIDTH_METERS = 8.052;

        /**
         * Point of Interest on the field
         * Contains a pose, alliance designation, and a descriptive tag
         */
        public static final class POI {
            private final Pose2d pose;
            private final String tag;
            private final String addr;

            public POI(Pose2d pose, String tag, String addr) {
                this.pose = pose;
                this.tag = tag;
                this.addr = addr;
            }

            public Pose2d get(Alliance alliance) {
                if (alliance == Alliance.Blue) {
                    return pose;
                } else {
                    return FlippingUtil.flipFieldPose(pose);
                }
            }

            /**
             * Gets the descriptive tag for this POI
             * 
             * @return The tag string
             */
            public String getTag() {
                return tag;
            }

            /**
             * Gets the addr for the POI
             * 
             * @return The addr string
             */
            public String getAddr() {
                return addr;
            }
        }

        /** All consolidated points of interest on the field */
        public static final POI[] ALL_POIS = {
                // Intake stations
                new POI(new Pose2d(1.25, 7.0, Rotation2d.fromDegrees(126.0)), "INTAKE_STATION", "left"),
                new POI(new Pose2d(1.25, FIELD_WIDTH_METERS - 7.0, Rotation2d.fromDegrees(234.0)), "INTAKE_STATION",
                        "right"),

                // Coral reef bars
                new POI(new Pose2d(5.9, 4.2, Rotation2d.fromDegrees(0)), "CORAL_REEF", "h"),
                new POI(new Pose2d(5.3, 5.15, Rotation2d.fromDegrees(60.0)), "CORAL_REEF", "i"),
                new POI(new Pose2d(5.0, 5.3, Rotation2d.fromDegrees(60.0)), "CORAL_REEF", "j"),
                new POI(new Pose2d(4.0, 5.3, Rotation2d.fromDegrees(120.0)), "CORAL_REEF", "k"),
                new POI(new Pose2d(3.7, 5.15, Rotation2d.fromDegrees(120.0)), "CORAL_REEF", "l"),
                new POI(new Pose2d(3.1, 4.2, Rotation2d.fromDegrees(180.0)), "CORAL_REEF", "a"),
                new POI(new Pose2d(5.9, FIELD_WIDTH_METERS - 4.2, Rotation2d.fromDegrees(0)), "CORAL_REEF", "g"),
                new POI(new Pose2d(5.3, FIELD_WIDTH_METERS - 5.15, Rotation2d.fromDegrees(300.0)), "CORAL_REEF", "f"),
                new POI(new Pose2d(5.0, FIELD_WIDTH_METERS - 5.3, Rotation2d.fromDegrees(300.0)), "CORAL_REEF", "e"),
                new POI(new Pose2d(4.0, FIELD_WIDTH_METERS - 5.3, Rotation2d.fromDegrees(230.0)), "CORAL_REEF", "d"),
                new POI(new Pose2d(3.7, FIELD_WIDTH_METERS - 5.15, Rotation2d.fromDegrees(240.0)), "CORAL_REEF", "c"),
                new POI(new Pose2d(3.1, FIELD_WIDTH_METERS - 4.2, Rotation2d.fromDegrees(180)), "CORAL_REEF", "b"),

                // Alga stations
                new POI(new Pose2d(6.0, 0.75, Rotation2d.fromDegrees(180.0)), "ALGA_STATION", "default"),

                // Cages
                new POI(new Pose2d(FIELD_LENGTH_METERS / 2, 7.265, Rotation2d.fromDegrees(0.0)), "CAGE", "left"),
                new POI(new Pose2d(FIELD_LENGTH_METERS / 2, 6.175, Rotation2d.fromDegrees(0.0)), "CAGE", "middle"),
                new POI(new Pose2d(FIELD_LENGTH_METERS / 2, 5.1, Rotation2d.fromDegrees(0.0)), "CAGE", "right")
        };

        /**
         * Legacy POI lists for backward compatibility - reference the consolidated list
         * by tag
         */
        public static final POI[] INTAKE_STATIONS = filterPoisByTag(ALL_POIS, "INTAKE_STATION");
        public static final POI[] CORAL_REEF_BARS = filterPoisByTag(ALL_POIS, "CORAL_REEF");
        public static final POI[] ALGA_STATIONS = filterPoisByTag(ALL_POIS, "ALGA_STATION");
        public static final POI[] CAGES = filterPoisByTag(ALL_POIS, "CAGE");

        /**
         * Filters the master POI list by tag
         * 
         * @param pois The array of POIs to filter
         * @param tag  The tag to match
         * @return Array of POIs with the matching tag
         */
        private static POI[] filterPoisByTag(POI[] pois, String tag) {
            return Arrays.stream(pois)
                    .filter(poi -> poi.getTag().equals(tag))
                    .toArray(POI[]::new);
        }

        /**
         * Finds a specific POI by its tag and address
         * 
         * @param tag     The tag to match
         * @param address The address value to match
         * @return The matching POI or null if not found
         */
        public static POI getPoiByTagAndAddress(String tag, String address) {
            return Arrays.stream(ALL_POIS)
                    .filter(poi -> poi.getTag().equals(tag) && poi.getAddr() == address)
                    .findFirst()
                    .orElse(null);
        }
    }
}
