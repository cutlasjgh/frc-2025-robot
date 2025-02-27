package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PickupPoints;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.CoralHandler;
import frc.robot.subsubsytems.LimitSwitch;
import java.util.function.BooleanSupplier;

public class PickupAndDeployCoralCommand extends SequentialCommandGroup {

    public PickupAndDeployCoralCommand() {
        Swerve swerve = Swerve.getInstance();
        CoralHandler coralHandler = CoralHandler.getInstance();

        // Get current robot pose.
        Pose2d currentPose = swerve.getPose();

        // Determine closest pickup point.
        Pose2d pickupTarget = getClosestPoint(PickupPoints.PICKUP_POINTS, currentPose);
        // Determine closest coral side point.
        Pose2d coralSideTarget = getClosestPoint(PickupPoints.CORAL_SIDE_POINTS, currentPose);

        addCommands(
            // Drive towards the pickup target while running coral intake.
            new ParallelCommandGroup(
                // Placeholder for drive command; assumes existence of driveToPoint(Translation2d)
                swerve.driveToPose(pickupTarget), 
                // Start coral intake.
                new IntakeCoralCommand().withTimeout(5) // timeout as a safeguard
            ),
            // Wait until the manipulator is at its limit (called on the subsystem)
            Commands.waitUntil(coralHandler::hasCoral),
            // Drive to the coral side target.
            swerve.driveToPose(coralSideTarget)
        );
    }

    // Helper method to pick the closest pose based on translation.
    private static Pose2d getClosestPoint(Pose2d[] points, Pose2d current) {
        Pose2d closest = points[0];
        double minDist = current.getTranslation().getDistance(closest.getTranslation());
        for (Pose2d p : points) {
            double d = current.getTranslation().getDistance(p.getTranslation());
            if (d < minDist) {
                minDist = d;
                closest = p;
            }
        }
        return closest;
    }
}
