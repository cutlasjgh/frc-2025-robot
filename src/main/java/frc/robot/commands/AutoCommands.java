package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.AlgaArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralManipulator;
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
        Swerve swerve = Swerve.getInstance();

        return Commands.run(() -> {
            // Drive backward at half speed (-0.5 m/s in the x direction)
            // Using setChassisSpeeds for robot-relative movement
            swerve.getSwerveDrive().setChassisSpeeds(
                    new ChassisSpeeds(-1.0, 0, 0) // x, y, rotation
            );
        }, swerve)
                .withTimeout(3.0)
                .andThen(() -> {
                    // Stop the robot
                    swerve.getSwerveDrive().setChassisSpeeds(
                            new ChassisSpeeds(0, 0, 0));
                });
    }

    public static Command leftToReef() {
        // Get subsystem instances
        Swerve swerve = Swerve.getInstance();
        CoralArm coralArm = CoralArm.getInstance();
        AlgaArm algaArm = AlgaArm.getInstance();
        CoralManipulator coralManipulator = CoralManipulator.getInstance();
        Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        Pose2d firstPose = FieldConstants.getPoiByTagAndAddress("CORAL_REEF", "i").get(alliance);
        Pose2d intakePose = FieldConstants.getPoiByTagAndAddress("INTAKE_STATION", "left").get(alliance);
        Pose2d secondPose = FieldConstants.getPoiByTagAndAddress("CORAL_REEF", "j").get(alliance);

        return Commands.sequence(
                // Run swerve.driveToPose once (scheduling the command) along with other actions
                Commands.parallel(
                        Commands.runOnce(() -> swerve.driveToPose(firstPose).schedule()),
                        coralArm.setMid(),
                        algaArm.release()),
                // Wait until robot velocity is low
                Commands.waitUntil(() -> {
                    ChassisSpeeds speeds = swerve.getRobotVelocity();
                    return Math.abs(speeds.vxMetersPerSecond) < 0.1 && Math.abs(speeds.vyMetersPerSecond) < 0.1;
                }),
                // Hold for 2 second to ensure complete stop
                Commands.waitSeconds(2.0),
                // Drop coral
                coralManipulator.drop().withTimeout(1.5),
                // Intake new coral
                Commands.parallel(
                        Commands.runOnce(() -> swerve.driveToPose(intakePose).schedule()),
                        Commands.waitSeconds(1.0).andThen(coralArm.setIntake()),
                        coralManipulator.intake()),
                // Drive back
                Commands.parallel(
                        Commands.runOnce(() -> swerve.driveToPose(secondPose).schedule()),
                        coralArm.setMid(),
                        algaArm.release()),
                // Wait until robot velocity is low
                Commands.waitUntil(() -> {
                    ChassisSpeeds speeds = swerve.getRobotVelocity();
                    return Math.abs(speeds.vxMetersPerSecond) < 0.1 && Math.abs(speeds.vyMetersPerSecond) < 0.1;
                }),
                // Hold for 2 second to ensure complete stop
                Commands.waitSeconds(2.0),
                // Drop coral
                coralManipulator.drop().withTimeout(1.5)).withName("leftToReef Auto");
    }

    public static Command rightToReef() {
        // Get subsystem instances
        Swerve swerve = Swerve.getInstance();
        CoralArm coralArm = CoralArm.getInstance();
        AlgaArm algaArm = AlgaArm.getInstance();
        CoralManipulator coralManipulator = CoralManipulator.getInstance();
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        Pose2d firstPose = FieldConstants.getPoiByTagAndAddress("CORAL_REEF", "f").get(alliance);
        Pose2d intakePose = FieldConstants.getPoiByTagAndAddress("INTAKE_STATION", "right").get(alliance);
        Pose2d secondPose = FieldConstants.getPoiByTagAndAddress("CORAL_REEF", "e").get(alliance);

        return Commands.sequence(
                // Run swerve.driveToPose once (scheduling the command) along with other actions
                Commands.parallel(
                        Commands.runOnce(() -> swerve.driveToPose(firstPose).schedule()),
                        coralArm.setMid(),
                        algaArm.release()),
                // Wait until robot velocity is low
                Commands.waitUntil(() -> {
                    ChassisSpeeds speeds = swerve.getRobotVelocity();
                    return Math.abs(speeds.vxMetersPerSecond) < 0.1 && Math.abs(speeds.vyMetersPerSecond) < 0.1;
                }),
                // Hold for 2 second to ensure complete stop
                Commands.waitSeconds(2.0),
                // Drop coral
                coralManipulator.drop().withTimeout(1.5),
                // Intake new coral
                Commands.parallel(
                        Commands.runOnce(() -> swerve.driveToPose(intakePose).schedule()),
                        Commands.waitSeconds(1.0).andThen(coralArm.setIntake()),
                        coralManipulator.intake()),
                // Drive back
                Commands.parallel(
                        Commands.runOnce(() -> swerve.driveToPose(secondPose).schedule()),
                        coralArm.setMid(),
                        algaArm.release()),
                // Wait until robot velocity is low
                Commands.waitUntil(() -> {
                    ChassisSpeeds speeds = swerve.getRobotVelocity();
                    return Math.abs(speeds.vxMetersPerSecond) < 0.1 && Math.abs(speeds.vyMetersPerSecond) < 0.1;
                }),
                // Hold for 2 second to ensure complete stop
                Commands.waitSeconds(2.0),
                // Drop coral
                coralManipulator.drop().withTimeout(1.5)).withName("rightToReef Auto");
    }
}
