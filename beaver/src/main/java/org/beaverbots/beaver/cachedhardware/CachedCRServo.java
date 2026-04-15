package org.beaverbots.beaver.cachedhardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class CachedCRServo implements CRServo {
    private final CRServo servo;

    private final double delta;

    private double lastPower = Double.NaN;

    public CachedCRServo(CRServo servo, double delta) {
        this.servo = servo;
        this.delta = delta;
    }

    @Override
    public void setPower(double power) {
        if (Math.abs(power - lastPower) > delta || (lastPower != 0 && power == 0) || Double.isNaN(lastPower)) {
            lastPower = power;
            servo.setPower(power);
        }
    }

    @Override
    public void setDirection(Direction direction) {
        if (getDirection() != direction) {
            lastPower = Double.NaN;
            servo.setDirection(direction);
        }
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
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
    public Direction getDirection() {
        return servo.getDirection();
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

    /// Custom extension to the CR servos, normally you would need (CRServoImplEx) cast and some object.
    public void setPwmRange(int lower, int upper) {
        ((CRServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(lower, upper));
    }
}
