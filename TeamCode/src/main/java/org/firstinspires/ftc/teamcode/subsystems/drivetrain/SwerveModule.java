package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.cachedhardware.CachedMotor;
import org.beaverbots.beaver.cachedhardware.CachedServo;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.teamcode.Transform;

public class SwerveModule implements Subsystem {
    CachedServo servo;
    CachedMotor motor;

    double currentAngle;

    public SwerveModule() {
        servo = new CachedServo(HardwareManager.claim(Servo.class, "swerveservo"), 0);
        motor = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "swervemotor"), 0);

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        servo.setDirection(Servo.Direction.FORWARD);

        currentAngle = servo.getPosition() * 2.0 * Math.PI;
    }

    public void drive(Transform velocity) {
        double angle = new Transform(0, 0).angleTo(velocity);
        double otherAngle = angle < Math.PI ? angle + Math.PI : angle - Math.PI;

        if (Math.abs(currentAngle - angle) < Math.abs(currentAngle - otherAngle)) {
            servo.setPosition(angle / 2.0 / Math.PI);
            motor.setPower(velocity.lateralDistance(new Transform(0, 0)));
        } else {
            servo.setPosition(otherAngle / 2.0 / Math.PI);
            motor.setPower(-velocity.lateralDistance(new Transform(0, 0)));
        }
    }
}
