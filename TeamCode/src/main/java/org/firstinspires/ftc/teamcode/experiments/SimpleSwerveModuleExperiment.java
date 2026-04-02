package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class SimpleSwerveModuleExperiment extends LinearOpMode {
    public void runOpMode() {
        CRServo servo1 = hardwareMap.get(CRServo.class, "servo1");
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        AnalogInput input1 = hardwareMap.get(AnalogInput.class, "input1");
        CRServo servo2 = hardwareMap.get(CRServo.class, "servo2");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        AnalogInput input2 = hardwareMap.get(AnalogInput.class, "input2");
        waitForStart();

        while (opModeIsActive()) {
            servo1.setPower(gamepad1.left_stick_x);
            motor1.setPower(gamepad1.left_stick_y);
            telemetry.addData("Servo 1 power", gamepad1.left_stick_x);
            telemetry.addData("Motor 1 power", gamepad1.left_stick_y);
            telemetry.addData("Servo 1 position", input1.getVoltage() / input1.getMaxVoltage() * 360);
            telemetry.addData("Motor 1 RPM", motor1.getVelocity() / 28 * 60);

            servo2.setPower(gamepad1.right_stick_x);
            motor2.setPower(gamepad1.right_stick_y);
            telemetry.addData("Servo 2 power", gamepad1.right_stick_x);
            telemetry.addData("Motor 2 power", gamepad1.right_stick_y);
            telemetry.addData("Servo 2 position", input2.getVoltage() / input2.getMaxVoltage() * 360);
            telemetry.addData("Motor 2 RPM", motor2.getVelocity() / 28 * 60);

            telemetry.update();
        }
    }
}
