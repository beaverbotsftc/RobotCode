package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="BeaverBots BasicTeleOp", group="Linear OpMode")
public class BeaverBotsTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor linearSlide = null;
    private Servo servo = null;
    private double speed = 0.5;
    private double turnspeed = 0.5;
    private double slideSpeed = 1;
    private double triggerspeed = 1;
    private int timesRepeated = 0;
    private DistanceSensor distanceSensor;
    private TouchSensor touchSensor;
    private ColorSensor colorSensor;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        startUp();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            timesRepeated += 1;

            if (gamepad1.left_bumper){
                speed = 0.25;
                turnspeed = 0.25;
            } else if (gamepad1.right_bumper) {
                speed = 1;
                turnspeed = 0.6;
            }else{
                speed = 0.5;
                turnspeed = 0.6;
            }

            if (gamepad2.dpad_up){
                servo.setPosition(0.42);
                // Closed Position
                // Tested all values under and this is the optimal one
                // If it's at anything lower it'll start vibrating
            } else if (gamepad2.dpad_down) {
                servo.setPosition(0.69);
                // Not tested just set to a good open position
            }

            if (gamepad1.right_trigger > 50){
                gamepad1.setLedColor(1,0,0,250);
            }else {
                gamepad1.setLedColor(0,1,0,250);
            }


            if(gamepad2.square){
                slideSpeed = 0.25;
                gamepad2.rumbleBlips(1);
            } else if (gamepad2.triangle) {
                slideSpeed = 0.50;
                gamepad2.rumbleBlips(1);
            } else if (gamepad2.circle) {
                slideSpeed = 0.75;
                gamepad2.rumbleBlips(1);
            } else if (gamepad2.cross) {
                slideSpeed = 1;
                gamepad2.rumbleBlips(1);
            }

            if (1-gamepad2.right_trigger <0.02){
                triggerspeed = 0.01;
            } else {
                triggerspeed = 1-gamepad2.right_trigger;
            }

            linearSlide.setPower(gamepad2.left_stick_y * slideSpeed * triggerspeed);

/*
            if(gamepad2.square){
                linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else if (gamepad2.triangle) {
                linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
*/
            double max;

            double axial   =  gamepad1.left_stick_y * -speed;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * speed;
            double yaw     =  gamepad1.right_stick_x * turnspeed;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = (axial + lateral + yaw);
            double rightFrontPower = (axial - lateral - yaw);
            double leftBackPower   = (axial - lateral + yaw);
            double rightBackPower  = (axial + lateral - yaw);

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

//            DistanceSensor();
//            ColorSensor();

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Times Looped: ", timesRepeated);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }

    public double DistanceSensor() {
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        boolean touch = touchSensor.isPressed();
        telemetry.addData("Distance:", distance);

        if (gamepad1.right_stick_button){
            if (distance <= 8.5) {
                telemetry.addLine("It very close brudda");
                if (!gamepad1.isRumbling()) {
                    gamepad1.rumbleBlips((int) distance);
                }
            }
        }

        if (touch) {
            telemetry.addLine("I'm being touched");
            if (!gamepad1.isRumbling()) {
                gamepad1.rumble(0, 1, 100);
            }
        }
        return distance;
    }

    public void ColorSensor() {
        colorSensor.enableLed(true);
        double colorRed = colorSensor.red();
        double colorGreen = colorSensor.green();
        double colorBlue = colorSensor.blue();

        telemetry.addLine("Color sensor things:");
        telemetry.addData("colorRed", colorRed);
        telemetry.addData("colorGreen", colorGreen);
        telemetry.addData("colorBlue", colorBlue);

        double colorSensorMultiplier = 0.9;
        if (colorRed * colorSensorMultiplier > colorBlue && colorRed * colorSensorMultiplier > colorGreen){
            telemetry.addLine("I see more red than the rest");
        }else if ((colorBlue * colorSensorMultiplier > colorRed && colorBlue * colorSensorMultiplier > colorGreen)) {
            telemetry.addLine("I see more blue than anything else");
        } else if ((colorGreen * colorSensorMultiplier > colorRed && colorGreen * colorSensorMultiplier > colorBlue)) {
            telemetry.addLine("I see more green than anything else");
        }else{
            telemetry.addLine("I don't see anything");
        }
    }

    private void startUp(){
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        servo = hardwareMap.get(Servo.class, "servo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        servo.setDirection(Servo.Direction.REVERSE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //linearSlide.getZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Wait for the game to start (driver presses PLAY)
    }
}
