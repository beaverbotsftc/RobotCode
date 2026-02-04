package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(group = "Experiments")
public class GateOpenerExperiment extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "gate servo");

        waitForStart();
        while (opModeIsActive()) {
            servo.setPosition(gamepad1.right_stick_y);
            telemetry.addData("Y", gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}
