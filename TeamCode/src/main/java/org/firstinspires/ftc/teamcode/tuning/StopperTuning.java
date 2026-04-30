package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.optimizedhardware.OptimizedServo;

@Autonomous
public class StopperTuning extends LinearOpMode {
    public void runOpMode() {
        Servo stopper = hardwareMap.get(Servo.class, "hood");

        double a = 0;
        waitForStart();
        while(opModeIsActive()) {
            a += gamepad1.left_stick_x * 0.0001;
            stopper.setPosition(a);
            telemetry.addData("a", a);
            telemetry.update();
        }
    }
}
