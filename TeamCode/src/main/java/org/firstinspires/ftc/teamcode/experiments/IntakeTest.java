package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(group = "Experiments")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "right back");

        waitForStart();

        while (!isStopRequested()) {
            intakeMotor.setPower(-gamepad1.right_stick_y);
        }
    }
}
