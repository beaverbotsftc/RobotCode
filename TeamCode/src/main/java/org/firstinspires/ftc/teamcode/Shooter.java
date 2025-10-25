package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Joystick Motor Control with Increment/Decrement + RPM", group="Examples")
public class Shooter extends LinearOpMode {

    private DcMotorEx motor;
    private DcMotorEx motor2;
    private boolean lastTriangle = false; // track last triangle state
    private boolean lastX = false;        // track last X state
    private double power = 0;             // persistent power value

    // Modern Robotics 6000 RPM motor specs
    private static final double MAX_RPM = 6000.0;
    private static final int TICKS_PER_REV = 28; // MR encoder spec

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "Motor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor2 = hardwareMap.get(DcMotorEx.class, "Motor2");
        motor2.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double speed = -gamepad1.left_stick_y;
            String preset = "Joystick";

            // Increment power when triangle is pressed
            if (gamepad1.triangle && !lastTriangle) {
                power += 0.05;
                if (power > 1) power = 1; // cap at max 1
                preset = "Incremented Power";
            }

            // Decrement power when X is pressed
            if (gamepad1.x && !lastX) {
                power -= 0.05;
                if (power < 0) power = 0; // minimum 0
                preset = "Decremented Power";
            }

            // Update last button states
            lastTriangle = gamepad1.triangle;
            lastX = gamepad1.x;

            // Apply motor power
            double outputPower = power * speed;
            motor.setPower(outputPower);
            motor2.setPower(outputPower);

            // Calculate RPMs
            double rpm1 = ticksPerSecondToRPM(motor.getVelocity());
            double rpm2 = ticksPerSecondToRPM(motor2.getVelocity());
            double encoder = motor.getCurrentPosition();
            double encoder2 = motor2.getCurrentPosition();

            // Telemetry
            telemetry.addData("Joystick Value", "%.2f", gamepad1.left_stick_y);
            telemetry.addData("Motor Power Limit", "%.2f", power);
            telemetry.addData("Active Preset", preset);
            telemetry.addData("Motor 1 RPM", "%.1f", rpm1);
            telemetry.addData("Motor 2 RPM", "%.1f", rpm2);
            telemetry.addData("Motor 1 encode", "%.1f", encoder);
            telemetry.addData("Motor 2 encode", "%.1f", encoder2);
            telemetry.update();
        }
    }

    private double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }
}
