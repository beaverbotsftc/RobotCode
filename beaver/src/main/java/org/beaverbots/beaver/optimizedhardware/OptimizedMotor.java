package org.beaverbots.beaver.optimizedhardware;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.beaverbots.beaver.command.CommandOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class OptimizedMotor implements DcMotorEx {
    private final DcMotorEx motor;

    private final double delta;
    private int readFrequency;
    private int readOffset;
    private int writeFrequency;
    private int writeOffset;

    // power -> [-1, 1]
    // velocity -> ticks/sec
    // target position -> encoder ticks
    private double lastPower = Double.NaN;
    private double lastWriteVelocityTicksPerSec = Double.NaN;
    private int lastTargetPosition = Integer.MIN_VALUE;
    private boolean hasLastTargetPosition = false;

    private double lastReadCurrentAmps = 0;
    private double lastReadVelocityTicksPerSec = 0;
    private int lastReadPosition = 0;

    private long lastCurrentReadLoop = -1;
    private long lastVelocityReadLoop = -1;
    private long lastPositionReadLoop = -1;

    public OptimizedMotor(DcMotorEx motor, double delta, int readFrequency, int readOffset, int writeFrequency, int writeOffset) {
        this.motor = motor;
        this.delta = delta;
        this.readFrequency = readFrequency;
        this.readOffset = readOffset;
        this.writeFrequency = writeFrequency;
        this.writeOffset = writeOffset;
    }

    public OptimizedMotor(DcMotorEx motor, double delta, int readFrequency, int readOffset) {
        this(motor, delta, readFrequency, readOffset, 1, 0);
    }

    public OptimizedMotor(DcMotorEx motor, double delta) {
        this(motor, delta, 1, 0, 1, 0);
    }

    public void setReadFrequency(int readFrequency) { this.readFrequency = readFrequency; }
    public void setReadOffset(int readOffset) { this.readOffset = readOffset; }
    public void setWriteFrequency(int writeFrequency) { this.writeFrequency = writeFrequency; }
    public void setWriteOffset(int writeOffset) { this.writeOffset = writeOffset; }

    private boolean canRead() {
        return (CommandOpMode.loopNumber + readOffset) % readFrequency == 0;
    }

    private boolean canWrite() {
        return (CommandOpMode.loopNumber + writeOffset) % writeFrequency == 0;
    }

    private double getTicksPerRev() {
        MotorConfigurationType motorType = motor.getMotorType();
        if (motorType == null) {
            return 1.0;
        }
        double ticksPerRev = motorType.getTicksPerRev();
        return ticksPerRev > 0 ? ticksPerRev : 1.0;
    }

    private double toTicksPerSecond(double angularRate, AngleUnit unit) {
        return unit.toDegrees(angularRate) / 360.0 * getTicksPerRev();
    }

    private double fromTicksPerSecond(double ticksPerSecond, AngleUnit unit) {
        return unit.fromDegrees(ticksPerSecond * 360.0 / getTicksPerRev());
    }

    @Override
    public void setPower(double power) {
        if (canWrite() &&
                (Double.isNaN(lastPower) || Math.abs(power - lastPower) > delta || (lastPower != 0 && power == 0))
        ) {
            lastPower = power;
            motor.setPower(power);
        }
    }

    @Override
    public void setVelocity(double angularRate) {
        if (canWrite()) {
            if (Double.isNaN(lastWriteVelocityTicksPerSec) || Math.abs(angularRate - lastWriteVelocityTicksPerSec) > delta) {
                lastWriteVelocityTicksPerSec = angularRate;
                motor.setVelocity(angularRate);
            }
        }
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        if (canWrite()) {
            double velocityTicksPerSec = toTicksPerSecond(angularRate, unit);
            if (Double.isNaN(lastWriteVelocityTicksPerSec) || Math.abs(velocityTicksPerSec - lastWriteVelocityTicksPerSec) > delta) {
                lastWriteVelocityTicksPerSec = velocityTicksPerSec;
                motor.setVelocity(angularRate, unit);
            }
        }
    }

    @Override
    public void setTargetPosition(int position) {
        if (canWrite() && (!hasLastTargetPosition || Math.abs((double) position - lastTargetPosition) > delta)) {
            hasLastTargetPosition = true;
            lastTargetPosition = position;
            motor.setTargetPosition(position);
        }
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        if ((canRead() || lastCurrentReadLoop < 0) && lastCurrentReadLoop != CommandOpMode.loopNumber) {
            lastReadCurrentAmps = motor.getCurrent(CurrentUnit.AMPS);
            lastCurrentReadLoop = CommandOpMode.loopNumber;
        }
        return unit.convert(lastReadCurrentAmps, CurrentUnit.AMPS);
    }

    @Override
    public double getVelocity() {
        if ((canRead() || lastVelocityReadLoop < 0) && lastVelocityReadLoop != CommandOpMode.loopNumber) {
            lastReadVelocityTicksPerSec = motor.getVelocity();
            lastVelocityReadLoop = CommandOpMode.loopNumber;
        }
        return lastReadVelocityTicksPerSec;
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return fromTicksPerSecond(getVelocity(), unit);
    }

    @Override
    public int getCurrentPosition() {
        if ((canRead() || lastPositionReadLoop < 0) && lastPositionReadLoop != CommandOpMode.loopNumber) {
            lastReadPosition = motor.getCurrentPosition();
            lastPositionReadLoop = CommandOpMode.loopNumber;
        }
        return lastReadPosition;
    }

    @Override
    public void setDirection(Direction direction) {
        if (getDirection() != direction) {
            // Direction affects the meaning of cached writes, so invalidate them.
            lastPower = Double.NaN;
            lastWriteVelocityTicksPerSec = Double.NaN;
            hasLastTargetPosition = false;
            motor.setDirection(direction);
        }
    }

    @Override
    public Direction getDirection() {
        return motor.getDirection();
    }

    @Override
    public double getPower() {
        return motor.getPower();
    }

    @Override
    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    @Override
    public void setMotorEnable() {
        motor.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        motor.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return motor.isMotorEnabled();
    }

    @Deprecated
    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        motor.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        motor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        motor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        motor.setPositionPIDFCoefficients(p);
    }

    @Deprecated
    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return motor.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return motor.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        motor.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return motor.getTargetPositionTolerance();
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return motor.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        motor.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return motor.getController();
    }

    @Override
    public int getPortNumber() {
        return motor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    @Deprecated
    @Override
    public void setPowerFloat() {
        motor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return motor.getPowerFloat();
    }

    @Override
    public boolean isBusy() {
        return motor.isBusy();
    }

    @Override
    public void setMode(RunMode mode) {
        motor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return motor.getMode();
    }

    @Override
    public Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        motor.close();
    }
}