package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class SimpleSwerveModuleExperiment extends LinearOpMode {
    public void runOpMode() {
        CRServo frontLeftServo = hardwareMap.get(CRServo.class, "front left servo");
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front left motor");
        AnalogInput frontLeftEncoder = hardwareMap.get(AnalogInput.class, "front left encoder");

        CRServo frontRightServo = hardwareMap.get(CRServo.class, "front right servo");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "front right motor");
        AnalogInput frontRightEncoder = hardwareMap.get(AnalogInput.class, "front right encoder");

        CRServo backLeftServo = hardwareMap.get(CRServo.class, "back left servo");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "back left motor");
        AnalogInput backLeftEncoder = hardwareMap.get(AnalogInput.class, "back left encoder");

        CRServo backRightServo = hardwareMap.get(CRServo.class, "back right servo");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "back right motor");
        AnalogInput backRightEncoder = hardwareMap.get(AnalogInput.class, "back right encoder");

        waitForStart();

        while (opModeIsActive()) {
            frontLeftServo.setPower(gamepad1.left_stick_x);
            frontLeftMotor.setPower(gamepad1.left_stick_y);
            telemetry.addData("Front Left Servo Power", gamepad1.left_stick_x);
            telemetry.addData("Front Left Motor Power", gamepad1.left_stick_y);
            telemetry.addData("Front Left Angle", frontLeftEncoder.getVoltage() / frontLeftEncoder.getMaxVoltage() * 360);
            telemetry.addData("Front Left RPM", frontLeftMotor.getVelocity() / 28 * 60);

            frontRightServo.setPower(gamepad1.right_stick_x);
            frontRightMotor.setPower(gamepad1.right_stick_y);
            telemetry.addData("Front Right Servo Power", gamepad1.right_stick_x);
            telemetry.addData("Front Right Motor Power", gamepad1.right_stick_y);
            telemetry.addData("Front Right Angle", frontRightEncoder.getVoltage() / frontRightEncoder.getMaxVoltage() * 360);
            telemetry.addData("Front Right RPM", frontRightMotor.getVelocity() / 28 * 60);

            backLeftServo.setPower(gamepad2.left_stick_x);
            backLeftMotor.setPower(gamepad2.left_stick_y);
            telemetry.addData("Back Left Servo Power", gamepad2.left_stick_x);
            telemetry.addData("Back Left Motor Power", gamepad2.left_stick_y);
            telemetry.addData("Back Left Angle", backLeftEncoder.getVoltage() / backLeftEncoder.getMaxVoltage() * 360);
            telemetry.addData("Back Left RPM", backLeftMotor.getVelocity() / 28 * 60);

            backRightServo.setPower(gamepad2.right_stick_x);
            backRightMotor.setPower(gamepad2.right_stick_y);
            telemetry.addData("Back Right Servo Power", gamepad2.right_stick_x);
            telemetry.addData("Back Right Motor Power", gamepad2.right_stick_y);
            telemetry.addData("Back Right Angle", backRightEncoder.getVoltage() / backRightEncoder.getMaxVoltage() * 360);
            telemetry.addData("Back Right RPM", backRightMotor.getVelocity() / 48 * 60);

            telemetry.update();
        }
    }
}