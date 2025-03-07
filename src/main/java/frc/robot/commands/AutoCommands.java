package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

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
}
