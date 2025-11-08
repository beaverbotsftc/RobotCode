package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group = "Experiments")
public class SingleMotorExperiment extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "right back");

        waitForStart();

        while (!isStopRequested()) {
            motor.setPower(-gamepad1.right_stick_y);
            telemetry.addData("Power", motor.getPower());
            telemetry.update();
        }
    }
}
