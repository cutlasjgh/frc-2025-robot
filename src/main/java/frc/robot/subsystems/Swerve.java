package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Manages the robot's swerve drive system, providing control over movement,
 * autonomous path following,
 * and odometry tracking. This subsystem handles both autonomous and
 * teleoperated drive control,
 * integrating with PathPlanner for advanced autonomous capabilities.
 * 
 * <p>
 * Features include:
 * <ul>
 * <li>Field-oriented drive control
 * <li>Autonomous path following and path finding
 * <li>Odometry tracking and pose estimation
 * <li>Wheel locking for stability
 * </ul>
 * 
 * <p>
 * Uses YAGSL (Yet Another Generic Swerve Library) for underlying swerve drive
 * implementation
 * and PathPlanner for autonomous navigation.
 */
public class Swerve extends SubsystemBase {
    private static Swerve instance;
    private final SwerveDrive drivebase;
    // private final DoubleArraySubscriber visionTranslation;

    /**
     * Returns the singleton instance of the Swerve subsystem.
     * Creates a new instance if one does not exist.
     * 
     * @return the Swerve subsystem instance
     */
    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    /**
     * Creates a new Swerve subsystem that manages drive control, path following,
     * and odometry.
     * Initializes the swerve drive with high telemetry verbosity and configures
     * various drive parameters.
     * 
     * @throws RuntimeException if swerve drive creation fails
     */
    public Swerve() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            drivebase = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(
                            RobotConstants.MAX_SPEED.in(MetersPerSecond),
                            new Pose2d(new Translation2d(Meter.of(8.774), Meter.of(4.026)),
                                    getAllianceRotation()));
        } catch (Exception e) {
            throw new RuntimeException("Failed to create swerve drive", e);
        }

        drivebase.setHeadingCorrection(true);
        drivebase.setAngularVelocityCompensation(true, true, 0.1);
        drivebase.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
        drivebase.setModuleEncoderAutoSynchronize(true, 1);
        drivebase.useExternalFeedbackSensor();

        setupPathPlanner();
    }

    /**
     * Returns the rotation to use based on the alliance color.
     * 0 degrees for blue alliance, 180 degrees for red alliance.
     *
     * @return Rotation2d set to 0 or 180 degrees based on alliance
     */
    private Rotation2d getAllianceRotation() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return Rotation2d.fromDegrees(180);
        }
        return Rotation2d.fromDegrees(0);
    }

    /**
     * Configures PathPlanner for autonomous path following.
     * Sets up the necessary callbacks and controllers for autonomous navigation,
     * including pose estimation, odometry reset, and velocity control.
     * Also initializes path finding warm-up for better initial performance.
     */
    public void setupPathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedForward = true;

            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotVelocity,
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedForward) {
                            drivebase.drive(
                                    speedsRobotRelative,
                                    drivebase.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces());
                        } else {
                            drivebase.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(5.0, 0.0, 0.0)),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);
        } catch (Exception e) {
            e.printStackTrace();
        }

        PathfindingCommand.warmupCommand().schedule();
    }

    /**
     * Creates a command to drive the robot in field-oriented mode.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * Swerve.getInstance().driveFieldOriented(() -> new ChassisSpeeds(1.0, 0.0, 0.5));
     * }
     * </pre>
     * 
     * @param velocity a supplier that provides the desired chassis speeds in
     *                 field-oriented coordinates
     * @return a command that continuously updates drive output based on supplied
     *         velocities
     * @see ChassisSpeeds
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> drivebase.driveFieldOriented(velocity.get()));
    }

    /**
     * Locks the swerve modules in an X pattern to prevent the robot from moving.
     * Useful for maintaining position or preparing for disable.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * Swerve.getInstance().lockWheels();
     * }
     * </pre>
     */
    public void lockWheels() {
        drivebase.lockPose();
    }

    /**
     * Resets the robot's odometry to the center of the field (8.774m, 4.026m),
     * with rotation based on alliance (0° for blue, 180° for red).
     * This is typically used at the start of autonomous routines.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * Swerve.getInstance().resetOdometry();
     * }
     * </pre>
     */
    public void resetOdometry() {
        drivebase.resetOdometry(new Pose2d(new Translation2d(Meter.of(8.774), Meter.of(4.026)),
                getAllianceRotation()));
    }

    /**
     * Retrieves the current estimated pose of the robot on the field.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * Pose2d currentPose = Swerve.getInstance().getPose();
     * }
     * </pre>
     * 
     * @return the current Pose2d representing the robot's position and rotation
     */
    public Pose2d getPose() {
        return drivebase.getPose();
    }

    /**
     * Resets the robot's odometry to a specific pose.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * Swerve.getInstance().resetOdometry(new Pose2d(new Translation2d(8.0, 4.0), Rotation2d.fromDegrees(90)));
     * }
     * </pre>
     * 
     * @param pose the Pose2d to set as the robot's current position and rotation
     */
    public void resetOdometry(Pose2d pose) {
        drivebase.resetOdometry(pose);
    }

    /**
     * Gets the current velocity of the robot.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * ChassisSpeeds speeds = Swerve.getInstance().getRobotVelocity();
     * }
     * </pre>
     * 
     * @return the ChassisSpeeds representing the robot's current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return drivebase.getRobotVelocity();
    }

    /**
     * Gets the underlying SwerveDrive object.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * SwerveDrive drive = Swerve.getInstance().getSwerveDrive();
     * }
     * </pre>
     * 
     * @return the SwerveDrive instance used by this subsystem
     */
    public SwerveDrive getSwerveDrive() {
        return drivebase;
    }

    /**
     * Creates a command to autonomously drive the robot to a specific pose using
     * PathPlanner.
     * <p>
     * Example:
     * 
     * <pre>
     * {@code
     * Command autoDrive = Swerve.getInstance()
     *         .driveToPose(new Pose2d(new Translation2d(6.0, 3.0), Rotation2d.fromDegrees(45)));
     * }
     * </pre>
     * 
     * @param pose the target Pose2d to drive to
     * @return a Command that will drive the robot to the specified pose
     */
    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
                drivebase.getMaximumChassisVelocity(), 4.0,
                drivebase.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                edu.wpi.first.units.Units.MetersPerSecond.of(0));
    }

}
