package frc.robot.subsubsytems;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Manages a single limit switch with interrupt-based detection.
 */
public class LimitSwitch {
    private final AsynchronousInterrupt interrupt;
    private final DigitalInput digitalInput;

    /**
     * Creates a new limit switch with callbacks for state changes.
     *
     * @param channel DIO channel for the switch
     * @param onPress Callback executed when switch is pressed (rising edge)
     * @param onRelease Callback executed when switch is released (falling edge)
     */
    public LimitSwitch(int channel, Runnable onPress, Runnable onRelease) {
        digitalInput = new DigitalInput(channel);
        interrupt = new AsynchronousInterrupt(digitalInput, (rising, falling) -> {
            if (rising && onPress != null) {
                onPress.run();
            }
            if (falling && onRelease != null) {
                onRelease.run();
            }
        });
        interrupt.enable();
    }

    /**
     * Creates a new limit switch with callback only for press.
     *
     * @param channel DIO channel for the switch
     * @param onPress Callback executed when switch is pressed
     */
    public LimitSwitch(int channel, Runnable onPress) {
        this(channel, onPress, null);
    }

    /**
     * @return true if switch is pressed
     */
    public boolean get() {
        return digitalInput.get();
    }
}
