package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what you are doing, do not modify this file except to
 * change the parameter class to the startRobot call.
 */
public final class Main {
  /**
   * Private constructor to prevent instantiation.
   * This class should not be instantiated since it only contains a static main method.
   */
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   * 
   * <p>If you change your main robot class, change the parameter type.
   *
   * @param args command line arguments (not used)
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
