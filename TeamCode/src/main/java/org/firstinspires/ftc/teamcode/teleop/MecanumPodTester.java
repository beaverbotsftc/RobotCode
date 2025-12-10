package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MecanumPodTester")
@Disabled
public class MecanumPodTester extends LinearOpMode {
    private final double slowSpeed = 0.6;
    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
//        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Motor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Motor2");

//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial   =  -changeInput(gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            double lateral =  changeInput(gamepad1.left_stick_x);
            double yaw     =  changeInput(gamepad1.right_stick_x);

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            double speed = gamepad1.right_bumper ? 1 : slowSpeed;

//            leftFrontDrive.setPower(speed * leftFrontPower);
            rightFrontDrive.setPower(speed * rightFrontPower);
//            leftBackDrive.setPower(speed * leftBackPower);
            rightBackDrive.setPower(speed * rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }

    private double changeInput(double x){
        return Math.signum(x) * (1 - Math.cos(x * Math.PI / 2.0));
    }
}
