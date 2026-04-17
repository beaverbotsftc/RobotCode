package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(group = "tuning")
public class TurretZeroTuning extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo leftTurret = hardwareMap.get(Servo.class, "left turret");
        Servo rightTurret = hardwareMap.get(Servo.class, "right turret");
        // No PWM reassignment needed because I'm setting it to 0.5.

        waitForStart();

        while (opModeIsActive()) {
            leftTurret.setPosition(0.5);
            rightTurret.setPosition(0.5);
        }
    }
}
