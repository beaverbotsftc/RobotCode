package org.beaverbots.beaver.cachedhardware;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CachedServo implements Servo {
    private final Servo servo;

    private final double delta;

    private double lastPosition = Double.NaN;

    public CachedServo(Servo servo, double delta) {
        this.servo = servo;
        this.delta = delta;
    }

    @Override
    public void setPosition(double position) {
        if (Math.abs(position - lastPosition) > delta || Double.isNaN(lastPosition)) {
            lastPosition = position;
            servo.setPosition(position);
        }
    }

    @Override
    public void setDirection(Direction direction) {
        if (direction != getDirection()) {
            lastPosition = Double.NaN;
            servo.setDirection(direction);
        }
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection();
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
    public double getPosition() {
        return servo.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
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
