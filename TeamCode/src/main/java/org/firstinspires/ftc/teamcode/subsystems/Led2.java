package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.cachedhardware.CachedServo;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;

public final class Led2 implements Subsystem {
    private CachedServo ledLeft;
    private CachedServo ledRight;

    private static final double RED = 0.277 + 0.001;
    private static final double VIOLET = 0.722 - 0.001;

    public Led2() {
        ledLeft = new CachedServo(HardwareManager.claim("led"), 0.01);
        ledRight = new CachedServo(HardwareManager.claim("led2"), 0.01);
    }

    public void setLeftHue(double hue) {
        ledLeft.setPosition(hue * (VIOLET - RED) + RED);
    }

    public void setRightHue(double hue) {
        ledRight.setPosition(hue * (VIOLET - RED) + RED);
    }

    public void turnOffLeft() {
        ledLeft.setPosition(0);
    }

    public void turnOffRight() {
        ledRight.setPosition(0);
    }
}