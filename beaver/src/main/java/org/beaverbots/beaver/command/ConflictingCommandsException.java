package org.beaverbots.beaver.command;

public class ConflictingCommandsException extends RuntimeException {
    public ConflictingCommandsException(String message) {
        super(message);
    }
}
