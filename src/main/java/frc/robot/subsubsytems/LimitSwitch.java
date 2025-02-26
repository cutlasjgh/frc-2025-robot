package frc.robot.subsubsytems;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Abstraction for limit switches with different backend implementations.
 * Supports both DIO and CANifier-based limit switches.
 *
 * <p>Example usage:
 * <pre>
 * {@code
 * // Create a limit switch using DIO:
 * LimitSwitch dioSwitch = LimitSwitch.createDIO(2,
 *         () -> System.out.println("DIO switch pressed"),
 *         () -> System.out.println("DIO switch released"));
 *
 * // Create a limit switch using CANifier:
 * CANifier canifier = new CANifier(1);
 * LimitSwitch canSwitch = LimitSwitch.createCANifier(canifier, CANifier.GeneralPin.kB,
 *         () -> System.out.println("CANifier switch pressed"),
 *         () -> System.out.println("CANifier switch released"));
 * }
 * </pre>
 */
public class LimitSwitch implements Supplier<Boolean> {
    /**
     * Enum representing the type of limit switch backend.
     */
    public enum LimitSwitchType {
        /** Digital Input/Output port on the RoboRIO */
        DIO,
        /** CANifier general purpose input */
        CANIFIER
    }
    
    /**
     * Interface for different limit switch hardware implementations.
     */
    private interface LimitSwitchBackend {
        /** Gets the current state of the limit switch */
        boolean get();
        
        /** Checks if this backend supports hardware interrupts */
        boolean supportsInterrupts();
        
        /** Sets up interrupts if supported */
        void setupInterrupts(Runnable onRising, Runnable onFalling);
        
        /** Updates the backend (for polling-based implementations) */
        void update();
        
        /** Gets the type of this backend */
        LimitSwitchType getType();
    }
    
    /**
     * Implementation for DIO-based limit switches with hardware interrupts.
     */
    private static class DIOBackend implements LimitSwitchBackend {
        private final DigitalInput digitalInput;
        private AsynchronousInterrupt interrupt;
        
        public DIOBackend(int channel) {
            digitalInput = new DigitalInput(channel);
        }
        
        @Override
        public boolean get() {
            return digitalInput.get();
        }
        
        @Override
        public boolean supportsInterrupts() {
            return true;
        }
        
        @Override
        public void setupInterrupts(Runnable onRising, Runnable onFalling) {
            interrupt = new AsynchronousInterrupt(digitalInput, (rising, falling) -> {
                if (rising && onRising != null) {
                    onRising.run();
                }
                if (falling && onFalling != null) {
                    onFalling.run();
                }
            });
            interrupt.enable();
        }
        
        @Override
        public void update() {
            // DIO uses hardware interrupts, no polling needed
        }
        
        @Override
        public LimitSwitchType getType() {
            return LimitSwitchType.DIO;
        }
    }
    
    /**
     * Implementation for CANifier-based limit switches.
     */
    private static class CANifierBackend implements LimitSwitchBackend {
        private final CANifier canifier;
        private final CANifier.GeneralPin pin;
        private boolean lastState = false;
        private Runnable onRising = null;
        private Runnable onFalling = null;
        
        public CANifierBackend(CANifier canifier, CANifier.GeneralPin pin) {
            this.canifier = canifier;
            this.pin = pin;
            this.lastState = get();
        }
        
        @Override
        public boolean get() {
            return !canifier.getGeneralInput(pin); // Assuming active low
        }
        
        @Override
        public boolean supportsInterrupts() {
            return false;
        }
        
        @Override
        public void setupInterrupts(Runnable onRising, Runnable onFalling) {
            this.onRising = onRising;
            this.onFalling = onFalling;
        }
        
        @Override
        public void update() {
            boolean currentState = get();
            if (currentState != lastState) {
                if (currentState && onRising != null) {
                    onRising.run();
                } else if (!currentState && onFalling != null) {
                    onFalling.run();
                }
                lastState = currentState;
            }
        }
        
        @Override
        public LimitSwitchType getType() {
            return LimitSwitchType.CANIFIER;
        }
    }
    
    private final LimitSwitchBackend backend;
    private static final List<LimitSwitchBackend> polledBackends = new ArrayList<>();
    
    /**
     * Private constructor used by factory methods.
     */
    private LimitSwitch(LimitSwitchBackend backend, Runnable onPress, Runnable onRelease) {
        this.backend = backend;
        
        if (backend.supportsInterrupts()) {
            backend.setupInterrupts(onPress, onRelease);
        } else {
            backend.setupInterrupts(onPress, onRelease);
            polledBackends.add(backend);
        }
    }
    
    /**
     * Creates a new limit switch with DIO backend.
     * 
     * @param channel DIO channel for the switch
     * @param onPress Callback executed when switch is pressed
     * @param onRelease Callback executed when switch is released
     * @return A new LimitSwitch instance
     */
    public static LimitSwitch createDIO(int channel, Runnable onPress, Runnable onRelease) {
        return new LimitSwitch(new DIOBackend(channel), onPress, onRelease);
    }
    
    /**
     * Creates a new limit switch with CANifier backend.
     * 
     * @param canifier CANifier device
     * @param pin GeneralPin to use for this limit switch
     * @param onPress Callback executed when switch is pressed
     * @param onRelease Callback executed when switch is released
     * @return A new LimitSwitch instance
     */
    public static LimitSwitch createCANifier(CANifier canifier, CANifier.GeneralPin pin, 
            Runnable onPress, Runnable onRelease) {
        return new LimitSwitch(new CANifierBackend(canifier, pin), onPress, onRelease);
    }
    
    /**
     * Creates the appropriate limit switch type based on the specified enum.
     * 
     * @param type Type of limit switch to create
     * @param dioChannel DIO channel if using DIO type (ignored for CANifier)
     * @param canifier CANifier instance if using CANifier type (ignored for DIO)
     * @param pin GeneralPin to use if using CANifier type (ignored for DIO)
     * @param onPress Callback executed when switch is pressed
     * @param onRelease Callback executed when switch is released
     * @return A new LimitSwitch instance
     */
    public static LimitSwitch create(LimitSwitchType type, int dioChannel, CANifier canifier, 
            CANifier.GeneralPin pin, Runnable onPress, Runnable onRelease) {
        switch (type) {
            case DIO:
                return createDIO(dioChannel, onPress, onRelease);
            case CANIFIER:
                return createCANifier(canifier, pin, onPress, onRelease);
            default:
                throw new IllegalArgumentException("Unknown limit switch type: " + type);
        }
    }
    
    /**
     * Creates a new limit switch with the appropriate backend based on type.
     * This is the recommended factory method for creating limit switches.
     * 
     * @param type The type of limit switch to create (DIO or CANIFIER)
     * @param config Configuration object containing all parameters for any type
     * @return A new LimitSwitch instance
     */
    public static LimitSwitch create(LimitSwitchConfig config) {
        return switch (config.type) {
            case DIO -> createDIO(config.dioChannel, config.onPress, config.onRelease);
            case CANIFIER -> createCANifier(config.canifier, config.pin, config.onPress, config.onRelease);
        };
    }
    
    /**
     * Configuration class to simplify limit switch creation.
     */
    public static class LimitSwitchConfig {
        public final LimitSwitchType type;
        public final int dioChannel;
        public final CANifier canifier;
        public final CANifier.GeneralPin pin;
        public final Runnable onPress;
        public final Runnable onRelease;
        
        /**
         * Creates a DIO limit switch configuration.
         */
        public static LimitSwitchConfig dio(int channel, Runnable onPress, Runnable onRelease) {
            return new LimitSwitchConfig(
                LimitSwitchType.DIO, 
                channel, 
                null, 
                null, 
                onPress, 
                onRelease
            );
        }
        
        /**
         * Creates a CANifier limit switch configuration.
         */
        public static LimitSwitchConfig canifier(
                CANifier canifier, 
                CANifier.GeneralPin pin, 
                Runnable onPress, 
                Runnable onRelease) {
            return new LimitSwitchConfig(
                LimitSwitchType.CANIFIER, 
                0, 
                canifier, 
                pin, 
                onPress, 
                onRelease
            );
        }
        
        private LimitSwitchConfig(
                LimitSwitchType type, 
                int dioChannel, 
                CANifier canifier, 
                CANifier.GeneralPin pin, 
                Runnable onPress, 
                Runnable onRelease) {
            this.type = type;
            this.dioChannel = dioChannel;
            this.canifier = canifier;
            this.pin = pin;
            this.onPress = onPress;
            this.onRelease = onRelease;
        }
    }
    
    /**
     * Gets the current state of the limit switch.
     * 
     * @return true if switch is pressed
     */
    @Override
    public Boolean get() {
        return backend.get();
    }
    
    /**
     * Gets the current state as a primitive boolean.
     * 
     * @return true if switch is pressed
     */
    public boolean getBoolean() {
        return backend.get();
    }
    
    /**
     * Gets the type of this limit switch.
     * 
     * @return The LimitSwitchType enum value
     */
    public LimitSwitchType getType() {
        return backend.getType();
    }
    
    /**
     * Updates all polling-based limit switches. Call from robot periodic method.
     */
    public static void updatePolledSwitches() {
        for (LimitSwitchBackend backend : polledBackends) {
            backend.update();
        }
    }

    /**
     * Adds callbacks to an existing limit switch.
     * Creates a new switch with the same backend but with the specified callbacks.
     * 
     * @param existingSwitch The existing limit switch
     * @param onPress Callback to execute when pressed
     * @param onRelease Callback to execute when released
     * @return A new limit switch with the same backend but with the specified callbacks
     */
    public static LimitSwitch addCallback(LimitSwitch existingSwitch, Runnable onPress, Runnable onRelease) {
        if (existingSwitch.getType() == LimitSwitchType.DIO) {
            // Create a new DIO switch with the same channel but with callbacks
            return createDIO(
                getDIOChannelFromSwitch(existingSwitch), 
                onPress, 
                onRelease
            );
        } else {
            // Create a new CANifier switch with the same pin but with callbacks
            return createCANifier(
                getCANifierFromSwitch(existingSwitch),
                getPinFromSwitch(existingSwitch),
                onPress,
                onRelease
            );
        }
    }
    
    // Helper methods for accessing private fields (would need to be implemented)
    private static int getDIOChannelFromSwitch(LimitSwitch sw) {
        // This would access a getter or field we'd need to add
        return 0; // Placeholder
    }
    
    private static CANifier getCANifierFromSwitch(LimitSwitch sw) {
        // This would access a getter or field we'd need to add
        return null; // Placeholder
    }
    
    private static CANifier.GeneralPin getPinFromSwitch(LimitSwitch sw) {
        // This would access a getter or field we'd need to add
        return null; // Placeholder
    }
}
