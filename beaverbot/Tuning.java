package org.firstinspires.ftc.teamcode.pathfollowerv3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Tuning")
public class Tuning extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, this::isStopRequested, telemetry);
    }
}