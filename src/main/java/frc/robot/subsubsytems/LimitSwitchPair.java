package frc.robot.subsubsytems;

import com.ctre.phoenix.CANifier;

import java.util.function.Supplier;

/**
 * Manages a pair of limit switches for a mechanism's min and max positions.
 *
 * <p>Example usage:
 * <pre>
 * {@code
 * // Create a LimitSwitchPair using DIO channels
 * LimitSwitchPair pair = new LimitSwitchPair(0, 1,
 *         () -> System.out.println("Min limit triggered"),
 *         () -> System.out.println("Max limit triggered"));
 * if (pair.isAtMin()) {
 *     // Handle min limit condition
 * }
 * }
 * </pre>
 */
public class LimitSwitchPair {
    private final Supplier<Boolean> minLimitSupplier;
    private final Supplier<Boolean> maxLimitSupplier;

    /**
     * Creates a new LimitSwitchPair with DIO channels.
     */
    public LimitSwitchPair(int minLimitChannel, int maxLimitChannel, 
            Runnable onMinTriggered, Runnable onMaxTriggered) {
        LimitSwitch minLimitSwitch = LimitSwitch.createDIO(minLimitChannel, onMinTriggered, null);
        LimitSwitch maxLimitSwitch = LimitSwitch.createDIO(maxLimitChannel, onMaxTriggered, null);
        
        minLimitSupplier = minLimitSwitch;
        maxLimitSupplier = maxLimitSwitch;
    }
    
    /**
     * Creates a new LimitSwitchPair with CANifier pins.
     */
    public LimitSwitchPair(CANifier canifier, CANifier.GeneralPin minLimitPin, CANifier.GeneralPin maxLimitPin,
            Runnable onMinTriggered, Runnable onMaxTriggered) {
        LimitSwitch minLimitSwitch = LimitSwitch.createCANifier(canifier, minLimitPin, onMinTriggered, null);
        LimitSwitch maxLimitSwitch = LimitSwitch.createCANifier(canifier, maxLimitPin, onMaxTriggered, null);
        
        minLimitSupplier = minLimitSwitch;
        maxLimitSupplier = maxLimitSwitch;
    }
    
    /**
     * Creates a new LimitSwitchPair with different backend types for min and max.
     */
    public LimitSwitchPair(LimitSwitch.LimitSwitchType minType, LimitSwitch.LimitSwitchType maxType,
            int minDioChannel, int maxDioChannel, CANifier canifier,
            CANifier.GeneralPin minPin, CANifier.GeneralPin maxPin,
            Runnable onMinTriggered, Runnable onMaxTriggered) {
        
        LimitSwitch minLimitSwitch = LimitSwitch.create(minType, minDioChannel, canifier, minPin, onMinTriggered, null);
        LimitSwitch maxLimitSwitch = LimitSwitch.create(maxType, maxDioChannel, canifier, maxPin, onMaxTriggered, null);
        
        minLimitSupplier = minLimitSwitch;
        maxLimitSupplier = maxLimitSwitch;
    }
    
    /**
     * Creates a new LimitSwitchPair with mixed suppliers.
     */
    public LimitSwitchPair(Supplier<Boolean> minLimitSupplier, Supplier<Boolean> maxLimitSupplier, 
            Runnable onMinTriggered, Runnable onMaxTriggered) {
        this.minLimitSupplier = minLimitSupplier;
        this.maxLimitSupplier = maxLimitSupplier;
        
        // For callbacks, we need to implement polling ourselves
        // This is handled by separate systems like Robot.periodic
    }

    /**
     * @return true if min limit is triggered
     */
    public boolean isAtMin() {
        return minLimitSupplier.get();
    }

    /**
     * @return true if max limit is triggered
     */
    public boolean isAtMax() {
        return maxLimitSupplier.get();
    }
}