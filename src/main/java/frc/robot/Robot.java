package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /** The autonomous command to run. */
  private Command autonomousCommand;

  /** The robot's container, which holds subsystems and commands. */
  private final RobotContainer robotContainer;

  /**
   * Creates a new Robot and initializes the RobotContainer.
   */
  public Robot() {
    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode.
   * Use this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** Called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** Called periodically when the robot is disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** Called once when disabled mode is exited. */
  @Override
  public void disabledExit() {
  }

  /**
   * Called when autonomous mode is initialized.
   * Schedules the autonomous command selected by the user.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** Called periodically during autonomous mode. */
  @Override
  public void autonomousPeriodic() {
  }

  /** Called once when autonomous mode is exited. */
  @Override
  public void autonomousExit() {
  }

  /**
   * Called when operator control is initialized.
   * Cancels the autonomous command if it's still running.
   */
  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** Called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** Called once when operator control is exited. */
  @Override
  public void teleopExit() {
  }

  /**
   * Called when test mode is initialized.
   * Cancels all running commands.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** Called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** Called once when test mode is exited. */
  @Override
  public void testExit() {
  }
}
