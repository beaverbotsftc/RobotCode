package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Swerve Module Tester")
public class SwerveModuleTester extends LinearOpMode {
    private final double slowSpeed = 0.5;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor = null;
    private Servo servo = null;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        motor  = hardwareMap.get(DcMotor.class, "Motor");
        servo  = hardwareMap.get(Servo.class, "servo");

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double speed = gamepad1.right_bumper ? 1 : slowSpeed;
            double pos = servo.getPosition();

            // Send calculated power to wheels
            motor.setPower(speed * -gamepad1.left_stick_y);
            servo.setPosition(gamepad1.triangle ? (pos + 0.003) : (gamepad1.cross ? (pos - 0.003) : pos));

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Wheel Power", motor.getPower());
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}
