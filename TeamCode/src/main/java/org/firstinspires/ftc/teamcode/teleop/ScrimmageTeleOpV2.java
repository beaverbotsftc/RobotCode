package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ScrimmageTeleOpV2")
public class ScrimmageTeleOpV2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx intake = null;

    private DcMotorEx motor;
    private DcMotorEx motor2;
    private Servo hoodServo;
    private DcMotorEx stopper;
    NormalizedColorSensor colorSensor;
    private double shootrpm = 2550;
    private static final int TICKS_PER_REV = 28;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left front");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right back");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor = hardwareMap.get(DcMotorEx.class, "shoot");
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        motor2.setDirection(DcMotor.Direction.FORWARD);

        hoodServo = hardwareMap.get(Servo.class, "hood");

        stopper = hardwareMap.get(DcMotorEx.class, "stopper");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        final float[] hsvValues = new float[3];
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color2");
        colorSensor.setGain(59);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        gamepad1.setLedColor(255,0,255, gamepad1.LED_DURATION_CONTINUOUS);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Color Sensor Detection
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            double dist = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

            if(gamepad1.right_bumper) {
                stopper.setPower(0.85);
            }else if (gamepad1.left_bumper){
                stopper.setPower(-0.85);
            }else{
                stopper.setPower(0);
            }

            //Intake
            if(gamepad1.right_trigger >= 0.05){
                if(stopper.getPower() < -0.5){
                    intake.setPower(0.8 * gamepad1.right_trigger);
                }else{
                    intake.setPower(gamepad1.right_trigger);
                }
            }else if(gamepad1.left_trigger >= 0.05){
                intake.setPower(-gamepad1.left_trigger);
            }else{
                intake.setPower(0);
            }


            //Hood Servo
            /*
            if(gamepad1.leftBumperWasPressed()){
                shootrpm = 2500;
                hoodServo.setPosition(0.0);
            }else if(gamepad1.rightBumperWasPressed()){
                shootrpm = 3500;
                hoodServo.setPosition(0.51);
            }

             */

            //Controls for shooter power
            if (gamepad1.triangle && shootrpm < 5000) {
                shootrpm += 5;
            }
            if (gamepad1.cross && shootrpm > 249) {
                shootrpm -= 5;
            }

            double rpm1 = ticksPerSecondToRPM(motor.getVelocity());
            double rpm2 = ticksPerSecondToRPM(motor2.getVelocity());
            double velocity = (rpm1+rpm2)/2.0;
            double motorPower = powerPiecewise(velocity);


            if (gamepad1.square) { //This is left back button
                motor.setPower(motorPower);
                motor2.setPower(motorPower);
            }else{
                motor.setPower(0);
                motor2.setPower(0);
            }

            if(motorPower != 0){
                if(Math.abs(velocity - shootrpm)/shootrpm <= 0.04){
                    gamepad1.rumble(0.45,0.45, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                }else{
                    gamepad1.stopRumble();
                }
            }

            double max;

            double axial   =  -changeInput(gamepad1.left_stick_y);
            double lateral =  changeInput(gamepad1.left_stick_x);
            double yaw     =  0.75*changeTurn(gamepad1.right_stick_x);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

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

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            telemetry.addLine("");
            telemetry.addData("Hood Pos", hoodServo.getPosition());


            // Telemetry
            telemetry.addData("Joystick Value", "%.2f", gamepad1.left_stick_y);
            telemetry.addData("Motor RPM Limit", "%.2f", shootrpm);
            telemetry.addData("Current Motor Power", "%.2f", motor.getPower());
            telemetry.addData("Motor 1 RPM", "%.1f", rpm1);
            telemetry.addData("Motor 2 RPM", "%.1f", rpm2);

            telemetry.addLine("");
            telemetry.addData("Current Intake Motor Power", "%.2f", intake.getPower());
            telemetry.addData("Intake RPM", "%.1f", intake.getVelocity());

            telemetry.update();
        }
    }

    private double changeInput(double x){
        return Math.signum(x) * (1 - Math.cos(x * Math.PI / 2.0));
    }

    private double changeTurn(double x){
        return 4.0*(Math.pow(x,5))/5.0 + x/5.0;
    }

    private double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    private double rpmToPower(double rpm) {return rpm/5140;}

    private double powerPiecewise(double rpm) {
        if (rpm < 300) {
            return 0.6 + (rpm * (0.4 / 300.0));
        } else if (rpm < 1500) {
            return 1.0;
        } else {
            return (rpm * -0.000358969756798) + 1.54;
        }
    }

}
