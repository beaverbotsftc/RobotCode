package org.firstinspires.ftc.teamcode.subsystems;

import org.beaverbots.beaver.cachedhardware.CachedServo;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.teamcode.Side;

public final class GateOpener implements Subsystem {
    public CachedServo redServo;
    public CachedServo blueServo;

    public GateOpener() {
        redServo = new CachedServo(HardwareManager.claim("left gate"), 0);
        blueServo = new CachedServo(HardwareManager.claim("right gate"), 0);

        close();
    }

    public void open() {
        redServo.setPosition(0.18);
        blueServo.setPosition(0.18);
    }

    public void close() {
        redServo.setPosition(0.5);
        blueServo.setPosition(0.5);
    }
}
