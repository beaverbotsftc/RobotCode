package org.firstinspires.ftc.teamcode.experiments;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class IntakeExperiment extends LinearOpMode {
    public void runOpMode() {
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "transfer1");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "transfer2");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            motor1.setPower(-gamepad1.left_stick_y);
            motor2.setPower(-gamepad1.left_stick_y);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Current", (motor1.getCurrent(CurrentUnit.AMPS) + motor2.getCurrent(CurrentUnit.AMPS)) / 2);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
