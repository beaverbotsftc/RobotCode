package org.firstinspires.ftc.teamcode.subsystems;

import org.beaverbots.beaver.cachedhardware.CachedServo;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.teamcode.Side;

public final class GateOpener implements Subsystem {
    public CachedServo servo;
    private Side side;

    public GateOpener(Side side) {
        servo = new CachedServo(HardwareManager.claim("gate servo"), 0);
        this.side = side;
    }

    public void open() {
        switch (side) {
            case RED:
                servo.setPosition(0);
                break;
            case BLUE:
                servo.setPosition(1);
                break;
        }
    }

    public void close() {
        servo.setPosition(0.5);
    }
}
