package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(group = "tuning")
public class SwerveServoOffsetTuning extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo frontLeftServo = hardwareMap.get(CRServo.class, "front left servo");
        CRServo backLeftServo = hardwareMap.get(CRServo.class, "back left servo");
        CRServo frontRightServo = hardwareMap.get(CRServo.class, "front right servo");
        CRServo backRightServo = hardwareMap.get(CRServo.class, "back right servo");

        frontLeftServo.setPower(0);
        backLeftServo.setPower(0);
        frontRightServo.setPower(0);
        backRightServo.setPower(0);

        AnalogInput frontLeftServoEncoder = hardwareMap.get(AnalogInput.class, "front left servo encoder");
        AnalogInput backLeftServoEncoder = hardwareMap.get(AnalogInput.class, "back left servo encoder");
        AnalogInput frontRightServoEncoder = hardwareMap.get(AnalogInput.class, "front right servo encoder");
        AnalogInput backRightServoEncoder = hardwareMap.get(AnalogInput.class, "back right servo encoder");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("front left", frontLeftServoEncoder.getVoltage() / frontLeftServoEncoder.getMaxVoltage() * 360);
            telemetry.addData("back left", backLeftServoEncoder.getVoltage() / backLeftServoEncoder.getMaxVoltage() * 360);
            telemetry.addData("front right", frontRightServoEncoder.getVoltage() / frontRightServoEncoder.getMaxVoltage() * 360);
            telemetry.addData("back right", backRightServoEncoder.getVoltage() / backRightServoEncoder.getMaxVoltage() * 360);
            telemetry.update();
        }
    }
}
