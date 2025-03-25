package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;

/** Subsystem that controls the climbing mechanism. */
public class Climb extends SubsystemBase {
  /** Singleton instance of the AlgaArm subsystem. */
  private static Climb instance;

  /**
   * Gets the singleton instance of the Climb subsystem.
   *
   * @return The singleton instance.
   */
  public static Climb getInstance() {
    if (instance == null) {
      instance = new Climb();
    }
    return instance;
  }

  private SparkMax climbMotor; // Removed final
  private DigitalInput topLimit; // Removed final
  private DigitalInput bottomLimit; // Removed final
  private NetworkTable table; // Removed final

  private Climb() {
    climbMotor = new SparkMax(ClimbConstants.CAN_ID, MotorType.kBrushless);
    topLimit = new DigitalInput(ClimbConstants.TOP_LIMIT_CHANNEL);
    bottomLimit = new DigitalInput(ClimbConstants.BOTTOM_LIMIT_CHANNEL);
    table = NetworkTableInstance.getDefault().getTable("Robot").getSubTable("Climb");

    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(IdleMode.kCoast);
    sparkMaxConfig.inverted(ClimbConstants.IS_INVERTED);
    sparkMaxConfig.smartCurrentLimit(30);
    climbMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Trigger that is active when the climb is at the top limit. */
  public final Trigger atTop = new Trigger(() -> !topLimit.get());

  /** Trigger that is active when the climb is at the bottom limit. */
  public final Trigger atBottom = new Trigger(() -> !bottomLimit.get());

  /**
   * Climbs until the top limit is reached.
   *
   * @return A command that climbs.
   */
  public Command climb() {
    return run(() -> {
          climbMotor.set(ClimbConstants.CLIMB_POWER);
        })
        .onlyWhile(atBottom.negate())
        .finallyDo(
            (interrupted) -> {
              stop().schedule();
            })
        .withName("climb");
  }

  /**
   * Stops the climb.
   *
   * @return A command that stops the climb.
   */
  public Command stop() {
    return runOnce(
            () -> {
              climbMotor.set(0);
            })
        .withName("stop");
  }

  @Override
  public void periodic() {
    table.getEntry("bottomLimit").setBoolean(atBottom.getAsBoolean());
    table.getEntry("topLimit").setBoolean(atTop.getAsBoolean());
  }
}
