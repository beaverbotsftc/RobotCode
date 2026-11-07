package org.firstinspires.ftc.teamcode.beaver.cachedhardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

public class CachedCRServo implements CRServo {
    private final CRServo servo;
    private final double delta;

    private double last;

    public CachedCRServo(CRServo servo, double delta) {
        this.servo = servo;
        this.delta = delta;
        this.last = 0;
    }

    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection();
    }

    @Override
    public void setPower(double power) {
        if (Math.abs(power - last) >= delta || (power == 0 && last != 0)) {
            servo.setPower(power);
            last = power;
        }
    }

    @Override
    public double getPower() {
        return servo.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return servo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return servo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return servo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        servo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        servo.close();
    }
}