package org.beaverbots.beaver.optimizedhardware;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.beaverbots.beaver.command.CommandOpMode;

public class OptimizedServo implements Servo {
    private final Servo servo;

    private final double delta;
    private int readFrequency;
    private int readOffset;
    private int writeFrequency;
    private int writeOffset;

    private double lastPosition = Double.NaN;
    private double lastReadPosition = 0;
    private long lastPositionReadLoop = -1;

    public OptimizedServo(Servo servo, double delta, int readFrequency, int readOffset, int writeFrequency, int writeOffset) {
        this.servo = servo;
        this.delta = delta;
        this.readFrequency = readFrequency;
        this.readOffset = readOffset;
        this.writeFrequency = writeFrequency;
        this.writeOffset = writeOffset;
    }

    public OptimizedServo(Servo servo, double delta, int readFrequency, int readOffset) {
        this(servo, delta, readFrequency, readOffset, 1, 0);
    }

    public OptimizedServo(Servo servo, double delta) {
        this(servo, delta, 1, 0, 1, 0);
    }

    public void setReadFrequency(int readFrequency) { this.readFrequency = readFrequency; }
    public void setReadOffset(int readOffset) { this.readOffset = readOffset; }
    public void setWriteFrequency(int writeFrequency) { this.writeFrequency = writeFrequency; }
    public void setWriteOffset(int writeOffset) { this.writeOffset = writeOffset; }

    private boolean canRead() {
        return (lastPositionReadLoop < 0) || (CommandOpMode.loopNumber + readOffset) % readFrequency == 0;
    }

    private boolean canWrite() {
        return (CommandOpMode.loopNumber + writeOffset) % writeFrequency == 0;
    }

    @Override
    public void setPosition(double position) {
        if (canWrite() && (Math.abs(position - lastPosition) > delta || Double.isNaN(lastPosition))) {
            lastPosition = position;
            servo.setPosition(position);
        }
    }

    @Override
    public double getPosition() {
        if (canRead() && lastPositionReadLoop != CommandOpMode.loopNumber) {
            lastReadPosition = servo.getPosition();
            lastPositionReadLoop = CommandOpMode.loopNumber;
        }
        return lastReadPosition;
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

    /**
     * Custom extension to the servos, normally you would need (ServoImplEx) cast.
     */
    public void setPwmRange(int lower, int upper) {
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(lower, upper));
        }
    }
}