package org.beaverbots.beaver.command;

public class AlreadyClaimedException extends RuntimeException {
    public AlreadyClaimedException(String message) {
        super(message);
    }
}
