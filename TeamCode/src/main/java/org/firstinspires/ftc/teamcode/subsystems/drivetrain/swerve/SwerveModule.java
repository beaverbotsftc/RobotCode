package org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.beaverbots.beaver.InfiniteServo;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.util.Geometry;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

public class SwerveModule implements Subsystem {
    private final InfiniteServo servo;
    private final DcMotorEx motor;
    private final Transform position;

    private final VoltageSensor voltageSensor;

    private Transform chassisTargetForce = Transform.ZERO;

    // Store the desired angle, defaulting to 0
    private double desired = 0;

    public static Transform getModuleTargetForce(Transform chassisForce, Transform wheelPosition) {
        Transform vLinear = chassisForce.lateral();
        // Angular component: Perpendicular to the radius vector (wheelPosition)
        Transform vAngular = new Transform(
                -chassisForce.getTheta() * wheelPosition.getY(),
                chassisForce.getTheta() * wheelPosition.getX()
        );
        return vLinear.add(vAngular);
    }

    public static double getWheelPower(Transform wheelPosition, double wheelDirection, Transform chassisTargetForce) {
        return getModuleTargetForce(chassisTargetForce, wheelPosition).dotLateral(Transform.FORWARD.rotateLateral(wheelDirection));
    }

    public SwerveModule(InfiniteServo servo, DcMotorEx motor, VoltageSensor voltageSensor, Transform position) {
        this.servo = servo;
        this.motor = motor;
        this.voltageSensor = voltageSensor;
        this.position = position;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(Transform force) {
        chassisTargetForce = force;
        if (force.lateralNorm() == 0 && force.getTheta() == 0) return;

        Transform projectedForce = getModuleTargetForce(force, position);

        double angle = Transform.ZERO.angleTo(projectedForce);
        double otherAngle = angle < Math.PI ? angle + Math.PI : angle - Math.PI;

        // 1. Calculate which path is closest to our CURRENT software setpoint (desired)
        // This eliminates jitter caused by hardware sensor noise
        double distAngleFromDesired = Math.abs(Geometry.normalizeAngle2(desired - angle));
        double distOtherFromDesired = Math.abs(Geometry.normalizeAngle2(desired - otherAngle));

        double targetAngle = (distAngleFromDesired <= distOtherFromDesired) ? angle : otherAngle;

        // 2. Check if the physical servo is dangerously far (> 120 degrees) from this calculated target
        double distFromActual = Math.abs(Geometry.normalizeAngle2(servo.getAngle() - targetAngle));

        if (distFromActual > Math.toRadians(120)) {
            // The servo is >120 degrees off from where we expect it to be.
            // Fall back to using the actual hardware servo position to decide the shortest path.
            double distAngleFromActual = Math.abs(Geometry.normalizeAngle2(servo.getAngle() - angle));
            double distOtherFromActual = Math.abs(Geometry.normalizeAngle2(servo.getAngle() - otherAngle));

            targetAngle = (distAngleFromActual <= distOtherFromActual) ? angle : otherAngle;
        }

        // 3. Apply and store the determined target
        servo.setAngle(targetAngle);
        desired = targetAngle;
    }

    public double getAngle() {
        return servo.getAngle();
    }

    public void periodic() {
        servo.periodic();

        double power = getWheelPower(position, servo.getAngle(), chassisTargetForce);

        motor.setPower(power);
    }
}