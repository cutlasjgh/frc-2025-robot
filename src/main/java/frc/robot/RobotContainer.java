package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;

/**
 * Main robot configuration class that binds controls and commands to
 * subsystems.
 * This class serves as the robot's command center, managing all subsystem
 * instances
 * and their associated commands.
 * 
 * <p>
 * Features include:
 * <ul>
 * <li>Driver control configuration
 * <li>Command button mappings
 * <li>Autonomous command selection
 * <li>Subsystem instantiation and management
 * </ul>
 * 
 * <p>
 * The class follows a centralized control pattern, with all robot behaviors
 * defined through command bindings and default commands.
 */
public class RobotContainer {
  /** Xbox controller used for driver input. */
  private final CommandXboxController driverController = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

  /** "Xbox" controller used for operator input. */
  private final CommandXboxController operatorController = new CommandXboxController(1);

  /** Main drive subsystem for robot movement. */
  private final Swerve swerveDrive = Swerve.getInstance();

  /**
   * Input stream for swerve drive control.
   * Configures how controller inputs are processed and applied to drive commands.
   */
  private final SwerveInputStream driveInputStream = SwerveInputStream.of(swerveDrive.getSwerveDrive(),
      () -> driverController.getLeftY() * -1,
      () -> driverController.getLeftX() * -1)
      .cubeTranslationControllerAxis(true)
      .scaleTranslation(0.5)
      .cubeRotationControllerAxis(true)
      .withControllerHeadingAxis(() -> driverController.getRightX() * -1, () -> driverController.getRightY() * -1)
      .cubeRotationControllerAxis(true)
      .deadband(OIConstants.DRIVER_DEADBAND)
      .allianceRelativeControl(true)
      .headingWhile(true);

  /** Alga arm subsystem for handling alga gamepieces. */
  private final AlgaArm algaArm = AlgaArm.getInstance();

  /** Coral handler subsystem for handling coral gamepieces. */
  private final CoralHandler coralHandler = CoralHandler.getInstance();

  /** Track the currently scheduled coral command, if any */
  private Command coralCommand = null;

  /** Track the currently scheduled alga command, if any */
  private Command algaCommand = null;

  /** Climb subsystem for handling climb mechanism. */
  private final Climb climb = Climb.getInstance();

  /**
   * Creates a new RobotContainer and initializes all robot subsystems and
   * commands.
   * Performs the following setup:
   * <ul>
   * <li>Silences joystick warnings for unplugged controllers
   * <li>Disables controller rumble
   * <li>Configures button bindings for commands
   * </ul>
   */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    driverController.setRumble(RumbleType.kBothRumble, 0.0);
    operatorController.setRumble(RumbleType.kBothRumble, 0.0);

    configureBindings();
  }

  /**
   * Configures button bindings for commands.
   * Maps controller buttons to specific robot actions:
   * <ul>
   * <li>Default command: Field oriented drive using joystick input
   * <li>X button: Lock wheels in X pattern for stability
   * <li>Start button: Reset odometry to field center
   * <li>Back button: Autonomous drive to field center
   * </ul>
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = swerveDrive.driveFieldOriented(driveInputStream);
    swerveDrive.setDefaultCommand(driveFieldOrientedDirectAngle);

    // driverController.back().whileTrue(
    // swerveDrive.driveToPose(
    // new Pose2d(new Translation2d(Meter.of(8.774), Meter.of(4.026)),
    // Rotation2d.fromDegrees(0))));

    // // A button: Intake/Drop Alga
    // operatorController.a().onTrue(algaArm.hasGamepiece() ?
    // new DropAlgaCommand() :
    // new IntakeAlgaCommand()
    // );

    // B button: Toggle the coral command (cancel running command on re-press)
    operatorController.b().onTrue(Commands.runOnce(() -> {
      if (coralCommand != null && coralCommand.isScheduled()) {
        coralCommand.cancel();
        coralCommand = null;
      } else {
        // Check the current state at press time
        if (coralHandler.hasCoral()) {
          coralCommand = new DropCoralCommand();
        } else {
          coralCommand = new IntakeCoralCommand();
        }
        coralCommand.schedule();
      }
    }));

    // A button: Toggle the alga command (cancel running command on re-press)
    operatorController.a().onTrue(Commands.runOnce(() -> {
      if (algaCommand != null && algaCommand.isScheduled()) {
        algaCommand.cancel();
        algaCommand = null;
      } else {
        // Check the current state at press time
        if (algaArm.hasGamepiece()) {
          algaCommand = new DropAlgaCommand();
        } else {
          algaCommand = new IntakeAlgaCommand();
        }
        algaCommand.schedule();
      }
    }));

    // Start button: Zero Coral mechanism if needed
    operatorController.y().onTrue(new ZeroCoralCommand(coralHandler));

    // X button: Run climb mechanism at 70% power while held
    driverController.x().whileTrue(new RunClimbCommand(climb));

    // D-Pad binding: operator dpad up -> go to TEST2 position
    operatorController.povUp().onTrue(
        new SetCoralPosition(coralHandler, CoralHandler.HandlerPosition.TEST2));
    // D-Pad binding: operator dpad left -> go to TEST1 position
    operatorController.povLeft().onTrue(
        new SetCoralPosition(coralHandler, CoralHandler.HandlerPosition.TEST1));
    // D-Pad binding: operator dpad down -> go to ZERO position
    operatorController.back().onTrue(
        new SetCoralPosition(coralHandler, CoralHandler.HandlerPosition.ZERO));
  }

  /**
   * Provides the command to run during autonomous mode.
   * Currently returns a placeholder command that prints a message,
   * indicating no autonomous routine is configured.
   *
   * @return the command to run in autonomous mode
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
