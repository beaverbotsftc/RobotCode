package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Random Axon Servo", group="Linear OpMode")
public class AxonServo extends LinearOpMode {
    private Servo rotationServo = null;

    @Override
    public void runOpMode() {
        rotationServo = hardwareMap.get(Servo.class, "servo");
        rotationServo.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                rotationServo.setPosition(0);
            } else if (gamepad1.dpad_right) {
                rotationServo.setPosition(0.33);
            } else if (gamepad1.dpad_down) {
                rotationServo.setPosition(0.66);
            } else if (gamepad1.dpad_left) {
                rotationServo.setPosition(1.0);
            }

            if (gamepad1.triangle){
                rotationServo.setPosition(rotationServo.getPosition() + 0.001);
            }else if(gamepad1.cross){
                rotationServo.setPosition(rotationServo.getPosition() - 0.001);
            }


        }
    }
}
