package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveInputStream;

public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);

  private final Swerve swerveDrive = Swerve.getInstance();

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

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    driver.setRumble(RumbleType.kBothRumble, 0.0);

    UsbCamera cam = CameraServer.startAutomaticCapture();
    cam.setVideoMode(PixelFormat.kYUYV, 176, 144, 30);

    configureBindings();

  }

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

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
