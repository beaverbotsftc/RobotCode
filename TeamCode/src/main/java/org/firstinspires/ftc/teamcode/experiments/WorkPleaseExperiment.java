package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class WorkPleaseExperiment extends LinearOpMode {
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "transfer2");
        //CRServo crServo = hardwareMap.get(CRServo.class, "front left servo");
        //AnalogInput input = hardwareMap.get(AnalogInput.class, "front left servo encoder");
        //Servo servo = hardwareMap.get(Servo.class, "stopper");

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(-gamepad1.left_stick_y);
            //crServo.setPower(-gamepad1.left_stick_y);
            //servo.setPosition(0.5 - gamepad1.left_stick_y / 2);

            telemetry.addData("Power", -gamepad1.left_stick_y);
            //telemetry.addData("Input", input.getVoltage());

            telemetry.update();
        }
    }
}
