package org.firstinspires.ftc.teamcode.beaver.command;

public class AlreadyClaimedException extends RuntimeException {
    public AlreadyClaimedException(String message) {
        super(message);
    }
}
