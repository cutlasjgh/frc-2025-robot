package frc.robot.helpers;

import com.revrobotics.spark.config.ClosedLoopConfig;

/** PID control constants container. */
public final class PID {
  /** Proportional gain. */
  public final double p;

  /** Integral gain. */
  public final double i;

  /** Derivative gain. */
  public final double d;

  /**
   * Creates new PID constants.
   *
   * @param p Proportional gain
   * @param i Integral gain
   * @param d Derivative gain
   */
  public PID(double p, double i, double d) {
    this.p = p;
    this.i = i;
    this.d = d;
  }

  /**
   * Applies the PID constants to a SparkMax configuration.
   * 
   * @param config The SparkMax configuration to apply the constants to.
   */
  public void applyToConfig(ClosedLoopConfig config) {
    config.pid(p, i, d);
  }
}
