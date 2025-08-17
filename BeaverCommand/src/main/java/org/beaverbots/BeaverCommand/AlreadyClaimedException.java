package org.beaverbots.BeaverCommand;

public class AlreadyClaimedException extends RuntimeException {
    public AlreadyClaimedException(String message) {
        super(message);
    }
}
