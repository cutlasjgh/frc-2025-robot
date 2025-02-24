package frc.robot.subsubsytems;

/**
 * Manages a pair of limit switches for minimum and maximum positions.
 */
public class LimitSwitchPair {
    private final LimitSwitch minSwitch;
    private final LimitSwitch maxSwitch;

    /**
     * Creates a new limit switch pair with callbacks for position limits.
     *
     * @param minChannel DIO channel for minimum position switch
     * @param maxChannel DIO channel for maximum position switch
     * @param onMin Callback executed when minimum position is reached
     * @param onMax Callback executed when maximum position is reached
     */
    public LimitSwitchPair(int minChannel, int maxChannel, Runnable onMin, Runnable onMax) {
        minSwitch = new LimitSwitch(minChannel, onMin);
        maxSwitch = new LimitSwitch(maxChannel, onMax);
    }

    /**
     * @return true if at minimum position
     */
    public boolean isAtMin() {
        return minSwitch.get();
    }

    /**
     * @return true if at maximum position
     */
    public boolean isAtMax() {
        return maxSwitch.get();
    }
}