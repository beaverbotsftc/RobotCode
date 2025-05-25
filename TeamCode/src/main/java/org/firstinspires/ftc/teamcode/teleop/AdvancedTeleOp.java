package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.collections.Controller;
import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.SubsystemsV2;


@TeleOp(name="Advanced TeleOp", group="Linear OpMode")
public class AdvancedTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Motors motors = new Motors();
    private SubsystemsV2 sub = new SubsystemsV2();
    private double speed = 0.5;
    private double turnspeed = 0.5;
    private double inverse = 1;
    private boolean isVerSlideEnabled = false;
    private boolean isHorSlideEnabled = false;
    private boolean isIntakeRotationEnabled = false;
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
        GOING_DOWN;
        public SampleStates increment() {
            return SampleStates.values()[(this.ordinal() + 1) % SampleStates.values().length];
        }

        public SampleStates decrement(){
            return SampleStates.values()[(this.ordinal() + SampleStates.values().length - 1) % SampleStates.values().length];
        }
    }

    public enum SpecimenPickState {
        // States to be defined
    }

    public enum SpecimenDropState {
        // States to be defined
    }
    private String activeState = "Sample";
    private SampleStates currentSampleState = SampleStates.TRAVEL;
    private SpecimenPickState currentSpecPickState; // Consider initializing
    private SpecimenDropState currentSpecDropState;


    @Override
    public void runOpMode() {
        motors.init(hardwareMap);
        sub.init(hardwareMap);
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
                    sub.verticalSlide(gamepad1.left_trigger);
                }else if(gamepad1.right_trigger > 0.05){
                    sub.verticalSlide(-gamepad1.right_trigger);
                }else{
                    sub.verticalSlide(0);
                }
            else if(isHorSlideEnabled){
                if(gamepad1.left_trigger > 0.05) {
                    sub.horizontalSlide(gamepad1.right_trigger);
                }else if(gamepad1.right_trigger > 0.05){
                    sub.horizontalSlide(-gamepad1.left_trigger);
                }else{
                    sub.horizontalSlide(0);
                }
            }

            if(controller.isSquareJustPressed()){
                currentSampleState = currentSampleState.increment();
                stateMachines();
            } else if (controller.isCrossJustPressed()) {
                currentSampleState = currentSampleState.decrement();
                stateMachines();
            }


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
            telemetry.addData("Horizontal Slide Power", sub.leftHorSlide.getPower());
            telemetry.addData("Intake Claw Servo Position", sub.intakeClawServo.getPosition());
            telemetry.addData("Intake Rotation Servo Position", sub.intakeRotationServo.getPosition());
            telemetry.addData("VerSlideEnabled", isVerSlideEnabled);
            telemetry.addData("HorSlideEnabled", isHorSlideEnabled);
            telemetry.addData("LeftVer Pos", sub.leftVerSlide.getCurrentPosition());
            telemetry.addData("LeftHor Pos", sub.leftHorSlide.getCurrentPosition());
            telemetry.addData("Intake Claw", sub.intakeClawServo.getPosition());
            telemetry.addData("Intake Wrist (RotServo)", sub.intakeRotationServo.getPosition());
            telemetry.addData("Intake Rotation (ArmServo)", sub.intakeArmServo.getPosition());
            telemetry.addData("Outtake Claw", sub.outtakeClawServo.getPosition());
            telemetry.addData("Outtake Wrist", sub.outtakeWristServo.getPosition());
            telemetry.addData("Outtake Rotation", sub.outtakeRotationServo.getPosition());
            telemetry.update();
        }
    }

    private void stateMachines() {
        switch (currentSampleState) {
            case TRAVEL:
                setControlFlags(false, false, false);
                sub.setOuttakeToTravelState();
                sub.setIntakeToTravelState();
                sub.powerOnOuttakeSubStatePos();
                sub.powerOnIntakeSubStatePos();
                break;
            case READY_TO_EXTEND:
                setControlFlags(true, false, false); // Horizontal slides enabled
                sub.powerOffOuttakeSubStatePos();
                sub.setIntakeSubStatePos(
                        sub.inPos.get("Claw Open"),
                        sub.inPos.get("Wrist Straight"),
                        sub.inPos.get("Rotation Straight")
                );
                sub.powerOnIntakeSubStatePos();
                break;
            case PICK_UP:
                setControlFlags(true, false, false); // Horizontal slides still enabled for pickup
                sub.powerOffOuttakeSubStatePos();
                sub.setIntakeSubStatePos(
                        sub.inPos.get("Claw Open"),
                        sub.inPos.get("Wrist Down"),
                        sub.inPos.get("Rotation Straight")
                );
                sub.powerOnIntakeSubStatePos();
                break;
            case PICKED_UP:
                setControlFlags(false, false, true); // Intake rotation variable, horizontal slides disabled
                sub.powerOffOuttakeSubStatePos();
                sub.setIntakeSubStatePos(
                        sub.inPos.get("Claw Close"),
                        sub.inPos.get("Wrist Down")
                );
                sub.powerOnIntakeSubStatePos();
                break;
            case RETRACTED:
                setControlFlags(false, false, false); // Slides retracting, no trigger control
                sub.setOuttakeToTravelState();
                sub.powerOnOuttakeSubStatePos();
                sub.setIntakeSubStatePos(
                        sub.inPos.get("Claw Close"),
                        sub.inPos.get("Wrist Up"),
                        sub.inPos.get("Rotation Straight")
                );
                sub.powerOnIntakeSubStatePos();
                // Note: subsystems.retractHorizontalSlides(); typically called on transition *to* this state
                break;
            case TRANSFER:
                setControlFlags(false, false, false);
                sub.setOuttakeSubStatePos(
                        sub.outPos.get("Claw Close"),
                        sub.outPos.get("Rotation Straight"),
                        sub.outPos.get("Wrist Down"),
                        sub.outPos.get("Arm Parallel Ground")
                );
                sub.powerOnOuttakeSubStatePos();
                sub.setIntakeSubStatePos(
                        sub.inPos.get("Claw Open"),
                        sub.inPos.get("Wrist Up"),
                        sub.inPos.get("Rotation Straight")
                );
                sub.powerOnIntakeSubStatePos();
                break;
            case DROP_OFF:
                setControlFlags(false, true, false); // Vertical slides enabled
                sub.setOuttakeSubStatePos(
                        sub.outPos.get("Claw Close"),
                        sub.outPos.get("Rotation Straight"),
                        sub.outPos.get("Wrist Up"),
                        sub.outPos.get("Arm Parallel Slides")
                );
                sub.powerOnOuttakeSubStatePos();
                sub.setIntakeToTravelState();
                sub.powerOnIntakeSubStatePos();
                break;
            case DROPPED_OFF:
                setControlFlags(false, true, false); // Vertical slides still enabled
                sub.setOuttakeSubStatePos(
                        sub.outPos.get("Claw Open"),
                        sub.outPos.get("Rotation Straight"),
                        sub.outPos.get("Wrist Up"),
                        sub.outPos.get("Arm Parallel Slides")
                );
                sub.powerOnOuttakeSubStatePos();
                sub.setIntakeToTravelState();
                sub.powerOnIntakeSubStatePos();
                break;
            case GOING_DOWN:
                setControlFlags(false, false, false); // Vertical slides retracting, no trigger control
                sub.setOuttakeSubStatePos(
                        sub.outPos.get("Claw Open"),
                        sub.outPos.get("Rotation Straight"),
                        sub.outPos.get("Wrist Down"),
                        sub.outPos.get("Arm Parallel Slides")
                );
                sub.powerOnOuttakeSubStatePos();
                sub.setIntakeToTravelState();
                sub.powerOnIntakeSubStatePos();
                // Note: subsystems.retractVerticalSlides(); typically called on transition *to* this state
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

    private void setControlFlags(boolean hor, boolean ver, boolean rotation){
        isHorSlideEnabled = hor;
        isVerSlideEnabled = ver; // Vertical Disabled for trigger control
        isIntakeRotationEnabled = rotation;
    }
}
