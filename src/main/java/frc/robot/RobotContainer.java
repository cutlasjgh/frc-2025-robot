package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.helpers.CustomSwerveInput;
import frc.robot.subsystems.*;

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
  /** Network table for robot-related bs */
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Robot");

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
  private final CustomSwerveInput driveInputStream = CustomSwerveInput.of(swerveDrive.getSwerveDrive(),
      () -> driverController.getLeftY() * -1,
      () -> driverController.getLeftX() * -1)
      .cubeTranslationControllerAxis(true)
      .scaleTranslation(0.75)
      .withControllerHeadingAxis(() -> driverController.getRightX() * -1, () -> driverController.getRightY() * -1)
      .cubeRotationControllerAxis(true)
      .deadband(OIConstants.DRIVER_DEADBAND)
      .allianceRelativeControl(true)
      .headingWhile(true);

  // private final SwerveInputStream driveInputStream =
  // SwerveInputStream.of(swerveDrive.getSwerveDrive(),
  // () -> driverController.getLeftY() * -1,
  // () -> driverController.getLeftX() * -1)
  // .cubeTranslationControllerAxis(true)
  // .scaleTranslation(1.0)
  // .withControllerRotationAxis(() -> driverController.getLeftX() * -1)
  // .cubeRotationControllerAxis(true)
  // .deadband(OIConstants.DRIVER_DEADBAND)
  // .allianceRelativeControl(true);

  /** Alga arm subsystem for handling alga gamepieces. */
  private final AlgaArm algaArm = AlgaArm.getInstance();

  /** Coral handler subsystem for handling coral gamepieces. */
  private final CoralManipulator coralManipulator = CoralManipulator.getInstance();

  /** Coral handler subsystem for handling coral gamepieces. */
  private final CoralArm coralArm = CoralArm.getInstance();

  /** Climb subsystem for handling climb mechanism. */
  private final Climb climb = Climb.getInstance();

  /** Apriltag subsystem for handling apriltags */
  private final Apriltag apriltag = Apriltag.getInstance();

  // Add the SendableChooser for autonomous
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

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
    configureAutoChooser();
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

    // This will be inverted in the future so that the driver does not rotate the robot and they have to override it
    driverController.leftBumper()
        .whileTrue(swerveDrive.driveFieldOriented(driveInputStream.copy().withHeading(() -> new Rotation2d(0))));

    driverController.povDown().whileTrue(swerveDrive.driveToClosestIntakeStation());
    driverController.povRight().whileTrue(swerveDrive.driveToClosestAlgaStation());
    driverController.povLeft().whileTrue(swerveDrive.driveToClosestCoralReefBar());
    driverController.x().whileTrue(climb.climb());

    operatorController.a().onTrue(algaArm.toggle());
    operatorController.leftBumper().whileTrue(algaArm.runIntake());
    operatorController.rightBumper().whileTrue(algaArm.runDrop());
    operatorController.b().onTrue(coralManipulator.toggle());

    operatorController.back().onTrue(Commands.runOnce(() -> coralArm.setZero().schedule()));
    operatorController.povRight().onTrue(Commands.runOnce(() -> coralArm.setIntake().schedule()));
    operatorController.povUp().onTrue(Commands.runOnce(() -> coralArm.setHigh().schedule()));
    operatorController.povDown().onTrue(Commands.runOnce(() -> coralArm.setLow().schedule()));
    operatorController.povLeft().onTrue(Commands.runOnce(() -> coralArm.setMid().schedule()));
    operatorController.start().onTrue(Commands.runOnce(() -> coralArm.setClimb().schedule()));
  }

  /**
   * Configure the autonomous command chooser with available options.
   */
  private void configureAutoChooser() {
    autoChooser.setDefaultOption("None", Commands.none());
    autoChooser.addOption("Simple Backward Drive", AutoCommands.simpleBackwardDrive());
    autoChooser.addOption("Mid to Reef", AutoCommands.midBlueToReef());
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Provides the command to run during autonomous mode.
   * Currently returns a placeholder command that prints a message,
   * indicating no autonomous routine is configured.
   *
   * @return the command to run in autonomous mode
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public NetworkTable getTable() {
    return table;
  }
}
