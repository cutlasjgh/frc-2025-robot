package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import org.json.simple.parser.ParseException;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Swerve;

import java.io.IOException;
import java.util.Optional;

/**
 * Container for autonomous command sequences.
 */
public class AutoCommands {

    private AutoCommands() {
        // Utility class - prevent instantiation
    }

    /**
     * Creates a command that drives the robot backward at half speed for 1 second.
     * 
     * @return The command to run
     */
    public static Command simpleBackwardDrive() {
        Swerve swerve = Swerve.getInstance();

        return Commands.run(() -> {
            // Drive backward at half speed (-0.5 m/s in the x direction)
            // Using setChassisSpeeds for robot-relative movement
            swerve.getSwerveDrive().setChassisSpeeds(
                    new ChassisSpeeds(-1.0, 0, 0) // x, y, rotation
            );
        }, swerve)
                .withTimeout(3.0) // Run for 1 second
                .andThen(() -> {
                    // Stop the robot
                    swerve.getSwerveDrive().setChassisSpeeds(
                            new ChassisSpeeds(0, 0, 0));
                });
    }

    /**
     * Creates a command that follows the "Mid to Reef" path and performs coral
     * actions.
     * This command works for both blue and red alliance - the PathPlanner
     * configuration
     * in Swerve.java handles the alliance-specific path mirroring automatically.
     * 
     * The sequence:
     * 1. Teleports the robot to the path's starting position
     * 2. Moves to "Mid to Reef" while positioning arm to mid
     * 3. Waits for coral to drop
     * 4. Returns arm to intake position
     *
     * @return The command to run
     */
    public static Command leftToReef() {
        // Get subsystem instances
        Swerve swerve = Swerve.getInstance();
        CoralArm coralArm = CoralArm.getInstance();
        AlgaArm algaArm = AlgaArm.getInstance();
        CoralManipulator coralManipulator = CoralManipulator.getInstance();

        // Define target POI pose
        Pose2d targetPose = new Pose2d(5.3, 5.15, Rotation2d.fromDegrees(60.0));

        return Commands.sequence(
            // Parallel phase:
            // Run swerve.driveToPose once (scheduling the command) along with other actions
            Commands.parallel(
                Commands.runOnce(() -> swerve.driveToPose(targetPose).schedule()),
                coralArm.setHigh(),
                algaArm.release()
            ),
            // Wait until robot velocity is low
            Commands.waitUntil(() -> {
                ChassisSpeeds speeds = swerve.getRobotVelocity();
                return Math.abs(speeds.vxMetersPerSecond) < 0.1 && Math.abs(speeds.vyMetersPerSecond) < 0.1;
            }),
            // Hold for 1 second to ensure complete stop
            Commands.waitSeconds(2.0),
            // Drop coral
            coralManipulator.drop().withTimeout(1.5)
        ).withName("leftToReef Auto");
    }
}
