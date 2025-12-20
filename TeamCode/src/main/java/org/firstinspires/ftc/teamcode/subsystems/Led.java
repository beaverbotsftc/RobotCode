package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;

public final class Led implements Subsystem {
    private Servo led;
    private double power;

    public Led() {
        this.led = HardwareManager.claim("led");
    }

    public void periodic() {
        led.setPosition(power);
    }

    public void turnOff() {
        this.power = 0.0;
    }

    public void setLed(double power){ this.power = power;}
    public void setLessPurple(){ this.power = 0.692;}
    public void setPurple(){ this.power = 0.721;}
}
