package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import org.json.simple.parser.ParseException;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.CoralArmConstants;
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
        return Commands.run(() -> {
            // Drive backward at half speed (-0.5 m/s in the x direction)
            // Using setChassisSpeeds for robot-relative movement
            Swerve.getInstance().getSwerveDrive().setChassisSpeeds(
                new ChassisSpeeds(-1.0, 0, 0)  // x, y, rotation
            );
        }, Swerve.getInstance())
        .withTimeout(3.0)  // Run for 1 second
        .andThen(() -> {
            // Stop the robot
            Swerve.getInstance().getSwerveDrive().setChassisSpeeds(
                new ChassisSpeeds(0, 0, 0)
            );
        });
    }
    
    /**
     * Creates a command that follows the "Mid to Reef" path and performs coral actions.
     * This command works for both blue and red alliance - the PathPlanner configuration
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
    public static Command midBlueToReef() {
        // Get subsystem instances
        Swerve swerve = Swerve.getInstance();
        CoralArm coralArm = CoralArm.getInstance();
        CoralManipulator coralManipulator = CoralManipulator.getInstance();
        
        // Get current alliance
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        
        // Default path name - configured for blue alliance (will be automatically flipped for red)
        String pathName = "Mid Blue to Reef";
        
        PathPlannerPath path;
        Command pathCommand;
        
        try {
            // Load the path
            path = PathPlannerPath.fromPathFile(pathName);
            
            // Create the PathPlanner path following command
            pathCommand = AutoBuilder.followPath(path);
        } catch (IOException | ParseException | AutoBuilderException e) {
            // Log the error
            DriverStation.reportError("Error loading path '" + pathName + "': " + e.getMessage(), e.getStackTrace());
            // Return a simple command that does nothing as fallback
            return Commands.sequence(
                Commands.runOnce(() -> DriverStation.reportWarning("Path '" + pathName + "' failed to load, using fallback", false)),
                // Simple fallback - just move arm to mid position and drop
                coralArm.setMid(),
                Commands.waitSeconds(1.0),
                coralManipulator.drop().withTimeout(1.5),
                coralArm.setIntake()
            ).withName("Mid to Reef - Fallback");
        }
        
        // Get the starting pose from the path (handle the Optional properly)
        Optional<Pose2d> startingPoseOptional = path.getStartingHolonomicPose();
        
        if (startingPoseOptional.isEmpty()) {
            DriverStation.reportWarning("Path '" + pathName + "' has no starting pose, using current robot position", false);
            // Use an alternative approach if there's no starting pose
            return Commands.sequence(
                // Run path following and arm movement to mid position in parallel
                Commands.parallel(
                    pathCommand,
                    coralArm.setMid()
                ),
                
                // After path is complete, drop coral
                coralManipulator.drop().withTimeout(1.5),
                
                // Move arm back to intake position
                coralArm.setIntake()
            ).withName("Mid to Reef Auto (No Reset)");
        }
        
        // Extract the actual pose from the Optional
        Pose2d startingPose = startingPoseOptional.get();
        
        return Commands.sequence(
            // First, teleport the robot to the path's starting position by resetting odometry
            Commands.runOnce(() -> {
                DriverStation.reportWarning("Teleporting robot to path start: " + startingPose + 
                    " (Alliance: " + alliance.orElse(DriverStation.Alliance.Blue) + ")", false);
                swerve.resetOdometry(startingPose);
            }),
            
            // Run path following and arm movement to mid position in parallel
            Commands.parallel(
                pathCommand,
                coralArm.setMid()
            ),
            
            // After path is complete, drop coral
            coralManipulator.drop().withTimeout(1.5),
            
            // Move arm back to intake position
            Commands.parallel(
                coralArm.setIntake(),
                coralManipulator.stop()
            )
        ).withName("Mid to Reef Auto (" + alliance.orElse(DriverStation.Alliance.Blue) + ")");
    }
}
