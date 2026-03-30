package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.InfiniteServo;
import org.beaverbots.beaver.cachedhardware.CachedCRServo;
import org.beaverbots.beaver.cachedhardware.CachedMotor;
import org.beaverbots.beaver.cachedhardware.CachedServo;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Transform;

import java.util.List;

public class SwerveModule implements Subsystem {
    InfiniteServo servo;
    DcMotorEx motor;

    public SwerveModule(InfiniteServo servo, DcMotorEx motor) {
        this.servo = servo;
        this.motor = motor;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void drive(Transform velocity) {
        double angle = new Transform(0, 0).angleTo(velocity);
        double otherAngle = angle < Math.PI ? angle + Math.PI : angle - Math.PI;

        if (Math.abs(servo.getAngle() - angle) < Math.abs(servo.getAngle() - otherAngle)) {
            servo.setAngle(angle);
            motor.setPower(velocity.lateralDistance(new Transform(0, 0)));
        } else {
            servo.setAngle(otherAngle);
            motor.setPower(-velocity.lateralDistance(new Transform(0, 0)));
        }
    }

    public void periodic() {
        servo.periodic();
    }
}
