package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.collections.Controller;
import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.SubsystemsV2;


@TeleOp(name="Basic TeleOp", group="Linear OpMode")
public class AdvancedTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Motors motors = new Motors();
    private SubsystemsV2 subsystems = new SubsystemsV2();
    private double speed = 0.5;
    private double turnspeed = 0.5;
    private double inverse = 1;
    private boolean isVerSlideEnabled = false;
    private boolean isHorSlideEnabled = false;
    private boolean isHanging = false;
    private Controller controller = new Controller();
    public enum SampleStates {
        TRAVEL,
        READY_TO_EXTEND,
        PICK_UP,
        PICKED_UP,
        RETRACTED,
        TRANSFER,
        DROP_OFF,
        DROPPED_OFF,
        GOING_DOWN
    }

    public enum SpecimenPickState {
        // States to be defined
    }

    public enum SpecimenDropState {
        // States to be defined
    }
    private SampleStates currentSampleState = SampleStates.TRAVEL;
    private SpecimenPickState currentSpecPickState; // Consider initializing
    private SpecimenDropState currentSpecDropState;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        motors.init(hardwareMap);
        subsystems.init(hardwareMap);
        controller.init(gamepad1, gamepad2);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            controller.update();
            speedControls();
//            inverseControls();

/*
            if(isVerSlideEnabled && gamepad1.left_trigger > 0.95 && gamepad1.right_trigger > 0.95) {
                if (!isHanging) {
                    subsystems.verticalSlide(-0.85);
                }
            } else if(isHanging && gamepad1.left_trigger < 0.1 && gamepad1.right_trigger > 0.95){
                subsystems.verticalSlide(-0.85);
            } else

 */

            if(isVerSlideEnabled)
                if(gamepad1.left_trigger > 0.05) {
                    subsystems.verticalSlide(gamepad1.left_trigger);
                }else if(gamepad1.right_trigger > 0.05){
                    subsystems.verticalSlide(-gamepad1.right_trigger);
                }else{
                    subsystems.verticalSlide(0);
                }
            else if(isHorSlideEnabled){
                if(gamepad1.left_trigger > 0.05) {
                    subsystems.horizontalSlide(gamepad1.right_trigger);
                }else if(gamepad1.right_trigger > 0.05){
                    subsystems.horizontalSlide(-gamepad1.left_trigger);
                }else{
                    subsystems.horizontalSlide(0);
                }
            }

            stateMachines();

            double max;

            double axial   =  -gamepad1.left_stick_y * speed * inverse;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * speed * inverse;
            double yaw     =  gamepad1.right_stick_x * turnspeed;

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

            motors.leftFrontDrive.setPower(leftFrontPower);
            motors.rightFrontDrive.setPower(rightFrontPower);
            motors.leftBackDrive.setPower(leftBackPower);
            motors.rightBackDrive.setPower(rightBackPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Horizontal Slide Power", subsystems.leftHorSlide.getPower());
            telemetry.addData("Intake Claw Servo Position", subsystems.intakeClawServo.getPosition());
            telemetry.addData("Intake Rotation Servo Position", subsystems.intakeRotationServo.getPosition());
            telemetry.addData("VerSlideEnabled", isVerSlideEnabled);
            telemetry.addData("HorSlideEnabled", isHorSlideEnabled);
            telemetry.addData("LeftVer Pos", subsystems.leftVerSlide.getCurrentPosition());
            telemetry.addData("LeftHor Pos", subsystems.leftHorSlide.getCurrentPosition());
            telemetry.addData("Intake Claw", subsystems.intakeClawServo.getPosition());
            telemetry.addData("Intake Wrist (RotServo)", subsystems.intakeRotationServo.getPosition());
            telemetry.addData("Intake Rotation (ArmServo)", subsystems.intakeArmServo.getPosition());
            telemetry.addData("Outtake Claw", subsystems.outtakeClawServo.getPosition());
            telemetry.addData("Outtake Wrist", subsystems.outtakeWristServo.getPosition());
            telemetry.addData("Outtake Rotation", subsystems.outtakeRotationServo.getPosition());
            telemetry.update();
            telemetry.update();
        }
    }

    private void stateMachines() {
        switch (currentSampleState) {
            case TRAVEL:
                break;
            case READY_TO_EXTEND:
                break;
            case PICK_UP:
                break;
            case PICKED_UP:
                break;
            case RETRACTED:
                break;
            case TRANSFER:
                break;
            case DROP_OFF:
                break;
            case DROPPED_OFF:
                break;
            case GOING_DOWN:
                break;
        }

        switch (currentSpecPickState) {
            // Cases for SpecimenPickState will go here
        }

        switch (currentSpecDropState) {
            // Cases for SpecimenDropState will go here
        }
    }

    private void inverseControls() {
        if(gamepad1.left_stick_button){
            inverse = 1;
            gamepad1.rumbleBlips(1);
        } else if (gamepad1.right_stick_button) {
            inverse = -1;
            gamepad1.rumbleBlips(1);
        }
    }

    private void speedControls() {
        if (gamepad1.left_bumper){
            speed = 0.25;
            turnspeed = 0.15;
        } else if (gamepad1.right_bumper) {
            speed = 1.0;
            turnspeed = 0.6;
        }else{
            speed = 0.55;
            turnspeed = 0.6;
        }
    }

}
