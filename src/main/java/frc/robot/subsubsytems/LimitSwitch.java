package frc.robot.subsubsytems;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
    
    // Static cache of DigitalInput objects to prevent duplicates
    private static final Map<Integer, DigitalInput> dioCache = new HashMap<>();
    
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
        private final int channel;
        private List<Runnable> onRisingCallbacks = new ArrayList<>();
        private List<Runnable> onFallingCallbacks = new ArrayList<>();
        
        public DIOBackend(int channel) {
            this.channel = channel;
            // Replace manual cache lookup with computeIfAbsent
            digitalInput = dioCache.computeIfAbsent(channel, DigitalInput::new);
        }
        
        @Override
        public boolean get() {
            return !digitalInput.get(); // Invert the raw reading for DIO
        }
        
        @Override
        public boolean supportsInterrupts() {
            return true;
        }
        
        @Override
        public void setupInterrupts(Runnable onRising, Runnable onFalling) {
            if (onRising != null) onRisingCallbacks.add(onRising);
            if (onFalling != null) onFallingCallbacks.add(onFalling);
            
            // Remove existing interrupt if there is one
            if (interrupt != null) {
                interrupt.disable();
                interrupt.close();
            }
            
            // Create new interrupt that calls all callbacks
            interrupt = new AsynchronousInterrupt(digitalInput, (rising, falling) -> {
                if (falling) { // Falling edge indicates switch is triggered
                    for (Runnable callback : onFallingCallbacks) {
                        callback.run();
                    }
                }
                if (rising) { // Rising edge indicates switch is cleared
                    for (Runnable callback : onRisingCallbacks) {
                        callback.run();
                    }
                }
            });
            interrupt.enable();
        }
        
        public int getChannel() {
            return channel;
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
        private List<Runnable> onRisingCallbacks = new ArrayList<>();
        private List<Runnable> onFallingCallbacks = new ArrayList<>();
        
        public CANifierBackend(CANifier canifier, CANifier.GeneralPin pin) {
            this.canifier = canifier;
            this.pin = pin;
            this.lastState = get();
        }
        
        @Override
        public boolean get() {
            // Note: Already inverted correctly
            return !canifier.getGeneralInput(pin); // Active low
        }
        
        @Override
        public boolean supportsInterrupts() {
            return false;
        }
        
        @Override
        public void setupInterrupts(Runnable onRising, Runnable onFalling) {
            if (onRising != null) onRisingCallbacks.add(onRising);
            if (onFalling != null) onFallingCallbacks.add(onFalling);
        }
        
        @Override
        public void update() {
            boolean currentState = get();
            if (currentState != lastState) {
                if (currentState) {
                    for (Runnable callback : onRisingCallbacks) {
                        callback.run();
                    }
                } else {
                    for (Runnable callback : onFallingCallbacks) {
                        callback.run();
                    }
                }
                lastState = currentState;
            }
        }
        
        public CANifier getCanifier() {
            return canifier;
        }
        
        public CANifier.GeneralPin getPin() {
            return pin;
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
     * Creates a new limit switch using a DIO channel.
     * <p>Example:
     * <pre>
     * {@code
     * LimitSwitch dioSwitch = LimitSwitch.createDIO(2,
     *         () -> System.out.println("Pressed"),
     *         () -> System.out.println("Released"));
     * }
     * </pre>
     */
    public static LimitSwitch createDIO(int channel, Runnable onPress, Runnable onRelease) {
        return new LimitSwitch(new DIOBackend(channel), onPress, onRelease);
    }

    /**
     * Creates a new limit switch using a CANifier.
     * <p>Example:
     * <pre>
     * {@code
     * CANifier canifier = new CANifier(1);
     * LimitSwitch canSwitch = LimitSwitch.createCANifier(canifier, CANifier.GeneralPin.kB,
     *         () -> System.out.println("Pressed"),
     *         () -> System.out.println("Released"));
     * }
     * </pre>
     */
    public static LimitSwitch createCANifier(CANifier canifier, CANifier.GeneralPin pin, 
            Runnable onPress, Runnable onRelease) {
        return new LimitSwitch(new CANifierBackend(canifier, pin), onPress, onRelease);
    }

    /**
     * Creates a limit switch based on the specified type.
     * <p>Example:
     * <pre>
     * {@code
     * LimitSwitch ls = LimitSwitch.create(LimitSwitch.LimitSwitchType.DIO, 2, null, null,
     *         () -> System.out.println("Pressed"),
     *         () -> System.out.println("Released"));
     * }
     * </pre>
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
     * Creates a limit switch using a configuration object.
     * <p>Example:
     * <pre>
     * {@code
     * LimitSwitchConfig config = LimitSwitchConfig.dio(2, 
     *         () -> System.out.println("Pressed"), 
     *         () -> System.out.println("Released"));
     * LimitSwitch ls = LimitSwitch.create(config);
     * }
     * </pre>
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
     * <p>Example:
     * <pre>
     * {@code
     * boolean pressed = ls.get();
     * }
     * </pre>
     */
    @Override
    public Boolean get() {
        return backend.get();
    }
    
    /**
     * Gets the current state as a primitive boolean.
     * <p>Example:
     * <pre>
     * {@code
     * boolean pressed = ls.getBoolean();
     * }
     * </pre>
     */
    public boolean getBoolean() {
        return backend.get();
    }
    
    /**
     * Gets the type of this limit switch.
     * <p>Example:
     * <pre>
     * {@code
     * LimitSwitchType type = ls.getType();
     * }
     * </pre>
     */
    public LimitSwitchType getType() {
        return backend.getType();
    }
    
    /**
     * Polls and updates all limit switches that require periodic polling.
     * <p>Example:
     * <pre>
     * {@code
     * LimitSwitch.updatePolledSwitches();
     * }
     * </pre>
     */
    public static void updatePolledSwitches() {
        for (LimitSwitchBackend backend : polledBackends) {
            backend.update();
        }
    }

    /**
     * Adds callbacks to an existing limit switch without creating a new DigitalInput.
     * <p>Example:
     * <pre>
     * {@code
     * LimitSwitch existingSwitch = LimitSwitch.createDIO(2, null, null);
     * existingSwitch.addCallbacks(
     *         () -> System.out.println("New pressed callback"),
     *         () -> System.out.println("New released callback"));
     * }
     * </pre>
     * 
     * @param onPress Callback to run when switch is pressed
     * @param onRelease Callback to run when switch is released
     * @return this switch instance for method chaining
     */
    public LimitSwitch addCallbacks(Runnable onPress, Runnable onRelease) {
        backend.setupInterrupts(onPress, onRelease);
        return this;
    }

    /**
     * Returns a new limit switch with additional callbacks.
     * <p>This method reuses the existing DigitalInput if possible to avoid resource conflicts.
     */
    public static LimitSwitch addCallback(LimitSwitch existingSwitch, Runnable onPress, Runnable onRelease) {
        return existingSwitch.addCallbacks(onPress, onRelease);
    }
    
    /**
     * Gets the DIO channel if this is a DIO switch.
     * @return the DIO channel, or -1 if not a DIO switch
     */
    public int getDIOChannel() {
        if (backend instanceof DIOBackend) {
            return ((DIOBackend) backend).getChannel();
        }
        return -1;
    }
    
    /**
     * Gets the CANifier if this is a CANifier switch.
     * @return the CANifier, or null if not a CANifier switch
     */
    public CANifier getCANifier() {
        if (backend instanceof CANifierBackend) {
            return ((CANifierBackend) backend).getCanifier();
        }
        return null;
    }
    
    /**
     * Gets the CANifier pin if this is a CANifier switch.
     * @return the pin, or null if not a CANifier switch
     */
    public CANifier.GeneralPin getPin() {
        if (backend instanceof CANifierBackend) {
            return ((CANifierBackend) backend).getPin();
        }
        return null;
    }
}
