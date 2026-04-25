package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ASigmaTeleOpFosho extends LinearOpMode {
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
    private double shootrpm = 0.4;
    private static final int TICKS_PER_REV = 28;
    private boolean shooterOn = false;

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
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   //     motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
   //     motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            stopper.setPower(gamepad1.right_bumper ? 0.8 :  gamepad1.left_bumper ? -0.8 : 0.0);

            //Intake
            intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);


            //Hood Servo
            if(gamepad1.dpad_up){
                hoodServo.setPosition(hoodServo.getPosition() + 0.001);
            //    hoodServo.setPosition(0.4256);
            }else if(gamepad1.dpad_down){
                hoodServo.setPosition(hoodServo.getPosition() - 0.001);
            //    hoodServo.setPosition(0.3928);
            }

            //close vals: 0.3628, 0.45
            //medium vals: 0.6711, 0.5
            if(gamepad1.dpad_up){
                hoodServo.setPosition(0.3628);
                shootrpm = 0.45;
            }else if(gamepad1.dpad_down){
                hoodServo.setPosition(0.6711);
                shootrpm = 0.51;
            }


/*
            //Controls for shooter power
            if (gamepad1.yWasPressed() && shootrpm < 1.0) {
                shootrpm += 0.05;
            }
            if (gamepad1.aWasPressed() && shootrpm > 0.0) {
                shootrpm -= 0.05;
            }

 */

            if(gamepad1.x){
                shooterOn = !shooterOn;
            }

            if (shooterOn) {
                motor.setPower(shootrpm);
                motor2.setPower(shootrpm);
            }else{
                motor.setPower(0);
                motor2.setPower(0);
            }



            /*
            if(shootrpm != 0){
                if(Math.abs(ticksPerSecondToRPM(motor.getVelocity()) - shootrpm)/shootrpm <= 0.035){
                    gamepad1.rumble(0.25,0.25, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                }else{
                    gamepad1.stopRumble();
                }
            }
             */

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
            telemetry.addData("Shooter Target RPM", "%.2f", shootrpm);
            telemetry.addData("Current Motor Power", "%.2f", motor.getPower());
            telemetry.addData("Motor 1 RPM", "%.1f", ticksPerSecondToRPM(motor.getVelocity()));

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

}
