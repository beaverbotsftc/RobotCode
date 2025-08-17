package org.beaverbots.BeaverCommand;

public class ConflictingCommandsException extends RuntimeException {
    public ConflictingCommandsException(String message) {
        super(message);
    }
}
