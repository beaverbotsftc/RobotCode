package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Random Axon Servo", group="Linear OpMode")
public class AxonServo extends LinearOpMode {
    private Servo openServo = null;
    private Servo rotationServo = null;

    @Override
    public void runOpMode() {
        openServo = hardwareMap.get(Servo.class, "open");
        rotationServo = hardwareMap.get(Servo.class, "rotation");
        openServo.setDirection(Servo.Direction.FORWARD);
        rotationServo.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                openServo.setPosition(0.40);
            } else if (gamepad1.dpad_right) {
                openServo.setPosition(0.41);
            } else if (gamepad1.dpad_down) {
                openServo.setPosition(0.42);
            } else if (gamepad1.dpad_left) {
                openServo.setPosition(0.55);
            }

            if(gamepad1.left_stick_y >= 0.05 && gamepad1.left_stick_y <= -0.05){
                rotationServo.setPosition(rotationServo.getPosition() + (gamepad1.left_stick_y / 1000));
            }


        }
    }
}
