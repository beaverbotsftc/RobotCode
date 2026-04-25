package org.beaverbots.beaver.optimizedhardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoController;

import org.beaverbots.beaver.command.CommandOpMode;

public class OptimizedCRServo implements CRServo {
    private final CRServo servo;

    private final double delta;
    private int readFrequency;
    private int readOffset;
    private int writeFrequency;
    private int writeOffset;

    private double lastPower = Double.NaN;
    private double lastReadPower = 0;
    private long lastPowerReadLoop = -1;

    public OptimizedCRServo(CRServo servo, double delta, int readFrequency, int readOffset, int writeFrequency, int writeOffset) {
        this.servo = servo;
        this.delta = delta;
        this.readFrequency = readFrequency;
        this.readOffset = readOffset;
        this.writeFrequency = writeFrequency;
        this.writeOffset = writeOffset;
    }

    public OptimizedCRServo(CRServo servo, double delta, int readFrequency, int readOffset) {
        this(servo, delta, readFrequency, readOffset, 1, 0);
    }

    public OptimizedCRServo(CRServo servo, double delta) {
        this(servo, delta, 1, 0, 1, 0);
    }

    public void setReadFrequency(int readFrequency) { this.readFrequency = readFrequency; }
    public void setReadOffset(int readOffset) { this.readOffset = readOffset; }
    public void setWriteFrequency(int writeFrequency) { this.writeFrequency = writeFrequency; }
    public void setWriteOffset(int writeOffset) { this.writeOffset = writeOffset; }

    private boolean canRead() {
        return (lastPowerReadLoop < 0) || (CommandOpMode.loopNumber + readOffset) % readFrequency == 0;
    }

    private boolean canWrite() {
        return (CommandOpMode.loopNumber + writeOffset) % writeFrequency == 0;
    }

    @Override
    public void setPower(double power) {
        if (canWrite() &&
                (Math.abs(power - lastPower) > delta || (lastPower != 0 && power == 0) || Double.isNaN(lastPower))
        ) {
            lastPower = power;
            servo.setPower(power);
        }
    }

    @Override
    public double getPower() {
        if (canRead() && lastPowerReadLoop != CommandOpMode.loopNumber) {
            lastReadPower = servo.getPower();
            lastPowerReadLoop = CommandOpMode.loopNumber;
        }
        return lastReadPower;
    }

    @Override
    public void setDirection(Direction direction) {
        if (getDirection() != direction) {
            lastPower = Double.NaN;
            servo.setDirection(direction);
        }
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection();
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

    /**
     * Custom extension to the CR servos, normally you would need (CRServoImplEx) cast.
     */
    public void setPwmRange(int lower, int upper) {
        if (servo instanceof CRServoImplEx) {
            ((CRServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(lower, upper));
        }
    }
}