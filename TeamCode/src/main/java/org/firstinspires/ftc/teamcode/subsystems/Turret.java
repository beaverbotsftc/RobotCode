package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.cachedhardware.CachedServo;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;

public class Turret implements Subsystem {
    private CachedServo turretLeft;
    private CachedServo turretRight;

    public Turret() {
        turretLeft = new CachedServo(HardwareManager.claim(Servo.class, "turret left"), 0.001);
        turretRight = new CachedServo(HardwareManager.claim(Servo.class, "turret right"), 0.001);
    }

    public void turn(double angle) {
        turretLeft.setPosition(angle / (2 * Math.PI));
        turretRight.setPosition(angle / (2 * Math.PI) - 0.125 / 100);
    }
}
