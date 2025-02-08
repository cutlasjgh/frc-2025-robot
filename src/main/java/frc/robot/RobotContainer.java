package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot's declarations should be.
 * It contains subsystems, commands, and button mappings that define the robot's behavior.
 */
public class RobotContainer {
  /** Xbox controller used for driver input. */
  private final CommandXboxController driver = new CommandXboxController(0);

  /** Main drive subsystem for robot movement. */
  private final Swerve swerveDrive = Swerve.getInstance();

  /** 
   * Input stream for swerve drive control.
   * Configures how controller inputs are processed and applied to drive commands.
   */
  private final SwerveInputStream driveInputStream = SwerveInputStream.of(swerveDrive.getSwerveDrive(),
      () -> driver.getLeftY() * -1,
      () -> driver.getLeftX() * -1)
      .cubeTranslationControllerAxis(true)
      .scaleTranslation(0.5)
      .cubeRotationControllerAxis(true)
      .withControllerHeadingAxis(() -> driver.getRightX() * -1, () -> driver.getRightY() * -1)
      .cubeRotationControllerAxis(true)
      .deadband(OIConstants.DRIVER_DEADBAND)
      .allianceRelativeControl(true)
      .headingWhile(true);

  /**
   * Creates a new RobotContainer.
   * The container initializes the subsystems, commands, and button bindings.
   */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    driver.setRumble(RumbleType.kBothRumble, 0.0);

    configureBindings();
  }

  /**
   * Configures button bindings for commands.
   * Maps controller buttons to specific robot actions:
   * - Default command: Field oriented drive
   * - X button: Lock wheels in X pattern
   * - Start button: Reset odometry
   * - Back button: Drive to field center
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = swerveDrive.driveFieldOriented(driveInputStream);
    swerveDrive.setDefaultCommand(driveFieldOrientedDirectAngle);

    driver.x().whileTrue(Commands.runOnce(swerveDrive::lockWheels, swerveDrive).repeatedly());
    driver.start().onTrue(Commands.runOnce(swerveDrive::resetOdometry, swerveDrive));
    driver.back().whileTrue(
        swerveDrive.driveToPose(
            new Pose2d(new Translation2d(Meter.of(8.774), Meter.of(4.026)),
                Rotation2d.fromDegrees(0))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
